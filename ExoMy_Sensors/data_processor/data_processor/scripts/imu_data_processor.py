import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import math

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        # Subscriber to listen to raw IMU data
        self.imu_sub = self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)

        # Publisher for processed IMU data
        self.processed_imu_pub = self.create_publisher(Float32MultiArray, 'processed_imu_data', 10)

        # Gyroscope biases (calculated from stationary data)
        self.gyro_x_bias = 1.4045801162719727
        self.gyro_y_bias = 1.14503812789917
        self.gyro_z_bias = 1.5648854970932007

        # Magnetometer biases (calculated from calibration data)
        # Replace these with the angles recorded from your phone's compass
        angle_x = 0  # Replace with the angle recorded when pointing the X-axis north
        angle_y = 0  # Replace with the angle recorded when pointing the Y-axis north
        angle_z = 0  # Replace with the angle recorded when pointing the Z-axis north

        # Calculate biases based on the recorded angles
        self.mag_x_bias = angle_x
        self.mag_y_bias = angle_y
        self.mag_z_bias = angle_z

    def imu_callback(self, msg: Imu):
        # Extract raw accelerometer data (in counts)
        accel_x_raw = msg.linear_acceleration.x
        accel_y_raw = msg.linear_acceleration.y
        accel_z_raw = msg.linear_acceleration.z

        # Extract raw gyroscope data (in counts) and subtract biases
        gyro_x_raw = msg.angular_velocity.x - self.gyro_x_bias
        gyro_y_raw = msg.angular_velocity.y - self.gyro_y_bias
        gyro_z_raw = msg.angular_velocity.z - self.gyro_z_bias

        # Extract raw magnetometer data (in counts) and subtract biases
        mag_x_raw = msg.orientation.x - self.mag_x_bias  # Using the orientation fields for magnetometer data
        mag_y_raw = msg.orientation.y - self.mag_y_bias
        mag_z_raw = msg.orientation.z - self.mag_z_bias

        # Normalize accelerometer data to m/s^2
        accel_x_normalized = accel_x_raw / 16384.0 * 9.81  # Assuming the accelerometer range is ±2g
        accel_y_normalized = accel_y_raw / 16384.0 * 9.81
        accel_z_normalized = accel_z_raw / 16384.0 * 9.81

        # Normalize gyroscope data (example scale: 32768, assuming full scale is ±250 degrees/s)
        gyro_x_normalized = gyro_x_raw / 131.0  # 32768 / 250 = 131
        gyro_y_normalized = gyro_y_raw / 131.0
        gyro_z_normalized = gyro_z_raw / 131.0

        # Normalize magnetometer data
        mag_x_normalized = mag_x_raw / 100.0  # Example scaling factor
        mag_y_normalized = mag_y_raw / 100.0
        mag_z_normalized = mag_z_raw / 100.0

        # Calculate roll, pitch, and yaw
        roll = math.atan2(accel_y_normalized, accel_z_normalized)
        pitch = math.atan2(-accel_x_normalized, math.sqrt(accel_y_normalized**2 + accel_z_normalized**2))
        yaw = math.atan2(mag_y_normalized, mag_x_normalized)

        # Convert roll, pitch, yaw to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # Log roll, pitch, yaw
        self.get_logger().info(f'Roll: {roll_deg:.2f} degrees')
        self.get_logger().info(f'Pitch: {pitch_deg:.2f} degrees')
        self.get_logger().info(f'Yaw: {yaw_deg:.2f} degrees')

        # Calculate magnetic north
        magnetic_north = math.degrees(math.atan2(mag_y_normalized, mag_x_normalized))
        self.get_logger().info(f'Magnetic North: {magnetic_north:.2f} degrees')

        # Create a matrix of processed IMU data (acceleration, gyroscope, magnetometer)
        imu_matrix = [
            accel_x_normalized, accel_y_normalized, accel_z_normalized,
            gyro_x_normalized, gyro_y_normalized, gyro_z_normalized,
            mag_x_normalized, mag_y_normalized, mag_z_normalized
        ]

        # Publish the processed IMU data
        imu_msg = Float32MultiArray(data=imu_matrix)
        self.processed_imu_pub.publish(imu_msg)

    def shutdown(self):
        self.get_logger().info("Shutting down IMU processor.")

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
