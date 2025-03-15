import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import ArducamDepthCamera as ac

class ToFPublisher(Node):
    def __init__(self):
        super().__init__('tof_publisher')
        self.tof_pub = self.create_publisher(Float32MultiArray, 'tof_data', 10)
        self.timer = self.create_timer(0.1, self.publish_tof_data)  # Publish every 100 ms
        self.cam = ac.ArducamCamera()

        # Try to open the camera
        ret = self.cam.open(ac.Connection.CSI, 0)
        if ret != 0:
            self.get_logger().error(f"Camera initialization failed with error code: {ret}")
            return

        # Start the camera
        ret = self.cam.start(ac.FrameType.RAW)
        if ret != 0:
            self.get_logger().error(f"Failed to start camera with error code: {ret}")
            self.cam.close()
            return

        self.get_logger().info("Arducam camera started successfully.")

    def publish_tof_data(self):
        # Request a frame from the camera
        frame = self.cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.RawData):
            buf = frame.raw_data
            self.cam.releaseFrame(frame)

            # Process the raw data (example: scaling it)
            buf = (buf / (1 << 4)).astype(np.uint8)  # Example scaling

            # Convert the buffer into a 2D array (matrix)
            tof_matrix = np.reshape(buf, (buf.shape[0] // 4, 4))  # Assuming you want a 4-column matrix

            # Create a Float32MultiArray message to publish the data
            tof_msg = Float32MultiArray(data=tof_matrix.flatten().tolist())

            # Publish the data
            self.tof_pub.publish(tof_msg)

    def shutdown(self):
        self.cam.stop()
        self.cam.close()

def main(args=None):
    rclpy.init(args=args)
    node = ToFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

