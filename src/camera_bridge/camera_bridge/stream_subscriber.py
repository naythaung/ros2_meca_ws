import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

WINDOWS_HOST_IP = "172.21.240.1"  # <-- update this
STREAM_URL = f"http://{WINDOWS_HOST_IP}:5000/video"


class StreamSubscriber(Node):
    def __init__(self):
        super().__init__('camera_stream_bridge')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        self.get_logger().info(f"Connecting to {STREAM_URL} ...")
        self.cap = cv2.VideoCapture(STREAM_URL)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video stream!")
        else:
            self.get_logger().info("Stream opened successfully.")

        timer_period = 1.0 / 30.0  # ~30 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StreamSubscriber()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
