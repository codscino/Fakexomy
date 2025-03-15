from argparse import ArgumentParser
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import TransformStamped
import tf2_ros
import qwiic_icm20948
import numpy as np
import time
from collections import deque

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # IMU and Magnetometer Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for publishing data
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish every 100 ms

        # Initialize the IMU sensor
        self.IMU = qwiic_icm20948.QwiicIcm20948()

        if not self.IMU.connected:
            self.get_logger().error("The Qwiic ICM20948 device isn't connected.")
            return

        self.IMU.begin()

        # Initialize position and velocity for translation (simple integration)
        self.position = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, vz]
        self.last_time = time.time()

        # Initialize deque for moving average filter
        self.accel_x_deque = deque(maxlen=5)
        self.accel_y_deque = deque(maxlen=5)
        self.accel_z_deque = deque(maxlen=5)

    def publish_imu_data(self):
        if self.IMU.dataReady():
            self.IMU.getAgmt()  # Read all axis and temp from sensor

            # IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Convert raw IMU values to SI units (acceleration in m/s² and angular velocity in rad/s)
            imu_msg.linear_acceleration.x = self.IMU.axRaw * 9.81 / 16384.0  # m/s²
            imu_msg.linear_acceleration.y = self.IMU.ayRaw * 9.81 / 16384.0
            imu_msg.linear_acceleration.z = self.IMU.azRaw * 9.81 / 16384.0

            imu_msg.angular_velocity.x = self.IMU.gxRaw * 3.14159 / 180 / 131.0  # rad/s
            imu_msg.angular_velocity.y = self.IMU.gyRaw * 3.14159 / 180 / 131.0
            imu_msg.angular_velocity.z = self.IMU.gzRaw * 3.14159 / 180 / 131.0

            # Fill covariance (adjust if needed)
            imu_msg.linear_acceleration_covariance[0] = -1  # Unknown
            imu_msg.angular_velocity_covariance[0] = -1
            imu_msg.orientation_covariance[0] = -1  # Orientation not provided

            self.imu_pub.publish(imu_msg)

            # Magnetometer message
            mag_msg = MagneticField()
            mag_msg.header.stamp = imu_msg.header.stamp
            mag_msg.header.frame_id = 'imu_link'

            mag_msg.magnetic_field.x = self.IMU.mxRaw * 1e-6  # Convert µT to T
            mag_msg.magnetic_field.y = self.IMU.myRaw * 1e-6
            mag_msg.magnetic_field.z = self.IMU.mzRaw * 1e-6

            self.mag_pub.publish(mag_msg)

            # Compute orientation and translation
            qx, qy, qz, qw = self.compute_orientation()
            self.compute_translation()

            # Publish the TF transform
            self.publish_tf(qx, qy, qz, qw)

        else:
            self.get_logger().info("Waiting for IMU data...")

    def compute_orientation(self):
        """Compute quaternion from accelerometer & magnetometer (basic tilt-compensation)."""
        ax, ay, az = self.IMU.axRaw, self.IMU.ayRaw, self.IMU.azRaw
        mx, my, mz = self.IMU.mxRaw, self.IMU.myRaw, self.IMU.mzRaw

        # Normalize accelerometer data
        norm_acc = np.sqrt(ax**2 + ay**2 + az**2)
        ax, ay, az = ax / norm_acc, ay / norm_acc, az / norm_acc

        # Compute roll and pitch
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # Compute yaw using magnetometer (simplified)
        yaw = np.arctan2(my, mx)

        # Convert roll, pitch, yaw to quaternion
        cy = np.cos(0.5 * yaw)
        sy = np.sin(0.5 * yaw)
        cr = np.cos(0.5 * roll)
        sr = np.sin(0.5 * roll)
        cp = np.cos(0.5 * pitch)
        sp = np.sin(0.5 * pitch)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def compute_translation(self):
        """Compute position and velocity using accelerometer data."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Integrate acceleration to get velocity and position
        ax_new = self.IMU.axRaw * 9.81 / 16384.0 # m/s²
        ay_new = self.IMU.ayRaw * 9.81 / 16384.0
        az_new = self.IMU.azRaw * 9.81 / 16384.0

        # Add current readings to deque
        self.accel_x_deque.append(ax_new)
        self.accel_y_deque.append(ay_new)
        self.accel_z_deque.append(az_new)

        # Compute moving average
        ax = np.mean(self.accel_x_deque)
        ay = np.mean(self.accel_y_deque)
        az = np.mean(self.accel_z_deque)

        # Compensate for gravity
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        gx = 10* np.sin(pitch)
        gy = 10 * np.sin(roll) * np.cos(pitch)
        gz = 10 * np.cos(roll) * np.cos(pitch)
        self.gravity = [gx, gy, gz]
        self.readings = [ax, ay, az]
        ax_affected = round(ax_new + gx ,2)
        ay_affected = round(ay_new - gy ,2)
        az_affected = round(az_new - gz ,2)
        self.get_logger().info(f"Readings: {self.readings},Gravity: {self.gravity}")
        self.get_logger().info(f"Acceleration: {ax_affected, ay_affected, az_affected}")
        


        # Update velocity (v = u + at)
        self.velocity[0] = ax_affected * dt
        self.velocity[1] = ay_affected * dt
        self.velocity[2] = az_affected * dt

        # Update position (s = ut + 1/2 at²)
        self.position[0] += self.velocity[0] * dt + 0.5 * ax_affected * dt**2
        self.position[1] += self.velocity[1] * dt + 0.5 * ay_affected * dt**2
        self.position[2] += self.velocity[2] * dt + 0.5 * az_affected * dt**2

        

        # Log position for debugging
        #self.get_logger().info(f"Readings: {self.readings},Gravity: {self.gravity} Position: {self.position}, Velocity: {self.velocity}")

    def publish_tf(self, qx, qy, qz, qw):
        """Publish IMU frame transformation with translation."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"  # Parent frame
        t.child_frame_id = "imu_link"  # Child frame
        
        # Publish translation along with rotation
        t.transform.translation.x = self.position[0]*0
        t.transform.translation.y = self.position[1]*0
        t.transform.translation.z = self.position[2]*0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def shutdown(self):
        self.get_logger().info("Shutting down IMU publisher.")

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__name__':
    main()
