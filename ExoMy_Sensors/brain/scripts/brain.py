import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Float32MultiArray
import numpy as np

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        # Subscribers to listen to processed IMU and ToF data
        self.imu_sub = self.create_subscription(Float32MultiArray, 'processed_imu_data', self.imu_callback, 10)
        self.tof_sub = self.create_subscription(Float32MultiArray, 'processed_tof_data', self.tof_callback, 10)

        # Publisher to send joystick data to the '/joy' topic
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)

        # Initialize a joystick message
        self.joy_msg = Joy()

    def imu_callback(self, msg: Float32MultiArray):
        # Extract processed IMU data (accelerometer + orientation)
        accel_x = msg.data[0]
        accel_y = msg.data[1]
        accel_z = msg.data[2]
        roll = msg.data[3]
        pitch = msg.data[4]
        yaw = msg.data[5]

        # Example: Set joystick axes based on roll, pitch, and acceleration (just as an example)
        self.joy_msg.axes = [roll, pitch, accel_x, accel_y, accel_z]
        self.joy_msg.buttons = [0, 0, 0]  # No button press in this example (can be set based on conditions)

        # Publish the joystick message
        self.joy_pub.publish(self.joy_msg)

    def tof_callback(self, msg: Float32MultiArray):
        # Process the ToF data if needed
        # Example: You can use the ToF data to control some behavior, e.g., distance thresholds
        tof_data = np.array(msg.data)

        # Example: Set joystick buttons based on proximity (distance)
        # Assuming tof_data contains some distance measurements, let's trigger button presses
        if np.mean(tof_data) < 0.5:  # Example threshold for distance
            self.joy_msg.buttons = [1, 0, 0]  # Button 0 pressed
        else:
            self.joy_msg.buttons = [0, 0, 0]  # No buttons pressed

        # Publish the joystick message
        self.joy_pub.publish(self.joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

