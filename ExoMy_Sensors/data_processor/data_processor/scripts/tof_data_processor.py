import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2

class ToFProcessor(Node):
    def __init__(self):
        super().__init__('tof_processor')

        # Subscriber to listen to raw ToF data
        self.tof_sub = self.create_subscription(Float32MultiArray, 'tof_data', self.tof_callback, 10)

        # Publisher for processed ToF data
        self.processed_tof_pub = self.create_publisher(Float32MultiArray, 'processed_tof_data', 10)

        # Video writer for saving the processed frames
        self.video_writer = cv2.VideoWriter('processed_tof_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (180, 100), True)

    def tof_callback(self, msg: Float32MultiArray):
        # Assuming the ToF data is a 2D matrix stored as a flat array
        tof_data = np.array(msg.data)

        # Reshape the data into a 2D matrix (240x180 resolution for the ToF camera)
        tof_matrix = np.reshape(tof_data, (180, 240))

        # Crop the borders (remove 50 pixels from the top and 30 pixels from each side)
        tof_matrix_cropped = tof_matrix[50:-30, 30:-30]

        # Filter out invalid or noisy depth values (e.g., values outside 0.5 to 2.0 meters)
        valid_mask = (tof_matrix_cropped >= 0.5) & (tof_matrix_cropped <= 1500)
        tof_matrix_cropped = np.where(valid_mask, tof_matrix_cropped, 1500)

        # Create the processed ToF data message
        processed_tof_data = tof_matrix_cropped.flatten().tolist()

        # Publish the processed data
        tof_msg = Float32MultiArray(data=processed_tof_data)
        self.processed_tof_pub.publish(tof_msg)

        # Normalize the matrix to the range 0-255 for 8-bit grayscale image
        tof_image = cv2.normalize(tof_matrix_cropped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Apply the viridis colormap
        tof_image_colored = cv2.applyColorMap(tof_image, cv2.COLORMAP_VIRIDIS)

        # Write the frame to the video
        self.video_writer.write(tof_image_colored)

    def shutdown(self):
        self.get_logger().info("Shutting down ToF processor.")
        self.video_writer.release()

def main(args=None):
    rclpy.init(args=args)
    node = ToFProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
