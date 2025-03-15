import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from threading import Thread
from argparse import ArgumentParser
from typing import Optional

from ArducamDepthCamera import (
    ArducamCamera,
    Connection,
    DeviceType,
    FrameType,
    Control,
    DepthData,
)

class Option:
    cfg: Optional[str]

class TOFPublisher(Node):
    def __init__(self, options: Option):
        super().__init__("tof_publisher")

        self.tof_ = self.__init_camera(options)
        if self.tof_ is None:
            raise Exception("Failed to initialize camera")

        self.pointsize_ = self.width_ * self.height_
        self.publisher_depth_ = self.create_publisher(Float32MultiArray, "raw_depth2", 1)
        self.running_ = True
        #self.timer_ = self.create_timer(1/3, self.publish_depth_data)  # 3 Hz
        self.timer_ = self.create_timer(1, self.publish_depth_data)  # 1 Hz
        self.depth_msg = Float32MultiArray()

    def __init_camera(self, options: Option):
        self.get_logger().info("Initializing ToF camera...")
        tof = ArducamCamera()
        ret = tof.open(Connection.CSI, 0) if options.cfg is None else tof.openWithFile(options.cfg, 0)

        if ret != 0:
            self.get_logger().error(f"Failed to open camera. Error code: {ret}")
            return None

        ret = tof.start(FrameType.DEPTH)
        if ret != 0:
            self.get_logger().error(f"Failed to start camera. Error code: {ret}")
            tof.close()
            return None

        info = tof.getCameraInfo()
        self.width_ = info.width
        self.height_ = info.height
        self.get_logger().info(f"Camera initialized. Resolution: {self.width_}x{self.height_}")
        return tof

    def publish_depth_data(self):
        frame = self.tof_.requestFrame(200)
        if frame is None or not isinstance(frame, DepthData):
            self.get_logger().warn("Failed to capture valid ToF depth frame!")
            return

        # Assume threshold for high confidence is 200 (adjust as necessary)


        # Convert depth and confidence data to numpy arrays
        depth_buf = np.array(frame.depth_data, dtype=np.float32).reshape(self.height_, self.width_)
        confidence_buf = np.array(frame.confidence_data, dtype=np.uint8).reshape(self.height_, self.width_)

        # Print min and max confidence
        min_confidence = np.min(confidence_buf)
        max_confidence = np.max(confidence_buf)
        confidence_threshold = max_confidence -15
        self.get_logger().info(f"Min confidence: {min_confidence}, Max confidence: {max_confidence}")

        # Remove 30 pixels from each side
        depth_buf = depth_buf[50:-50, 50:-50]
        confidence_buf = confidence_buf[50:-50, 50:-50]
        self.get_logger().info(f"{depth_buf.size}")


        # Mask the depth buffer to only include points with high confidence
        high_confidence_mask = confidence_buf >= confidence_threshold
        filtered_depth_buf = np.where(high_confidence_mask, depth_buf, np.nan)  # Replace low confidence depths with NaN

        # Flatten the filtered depth data and publish
        depth_data_to_send = filtered_depth_buf.flatten().tolist()
        #depth_data_to_send[::2] = np.full_like(depth_data_to_send[::2], np.nan)
        self.depth_msg.data = depth_data_to_send
        self.publisher_depth_.publish(self.depth_msg)

        # Release the frame
        self.tof_.releaseFrame(frame)

    def stop(self):
        self.running_ = False
        self.tof_.stop()
        self.tof_.close()


def main(args=None):
    rclpy.init(args=args)
    parser = ArgumentParser()
    parser.add_argument("--cfg", type=str, help="Path to camera configuration file")
    ns = parser.parse_args()

    options = Option()
    options.cfg = ns.cfg

    tof_publisher = TOFPublisher(options)
    rclpy.spin(tof_publisher)

    tof_publisher.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
