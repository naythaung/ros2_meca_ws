import os
import csv
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import tf2_ros

OUTPUT_DIR = os.path.expanduser('~/meca_ws/captures')
WORLD_FRAME = 'world'
CAMERA_FRAME = 'camera_optical_frame'
CAPTURE_INTERVAL_S = 1.0  # how often to auto-capture, in seconds


class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.csv_path = os.path.join(OUTPUT_DIR, 'poses.csv')
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'index', 'filename', 'stamp_sec', 'stamp_nanosec',
                    'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
                ])

        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.capture_count = self._count_existing_captures()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10
        )
        self.srv = self.create_service(
            Trigger, 'capture_frame', self.capture_callback
        )

        self.auto_timer = self.create_timer(CAPTURE_INTERVAL_S, self.auto_capture)

        self.get_logger().info(
            f"Capture node ready. Auto-capturing every {CAPTURE_INTERVAL_S}s to {OUTPUT_DIR}."
        )

    def _count_existing_captures(self):
        existing = [f for f in os.listdir(OUTPUT_DIR) if f.startswith('capture_') and f.endswith('.png')]
        return len(existing)

    def image_callback(self, msg):
        self.latest_image_msg = msg

    def do_capture(self):
        if self.latest_image_msg is None:
            return False, "No image received yet."

        img_msg = self.latest_image_msg
        stamp = img_msg.header.stamp

        try:
            transform = self.tf_buffer.lookup_transform(
                WORLD_FRAME, CAMERA_FRAME, Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            return False, f"TF lookup failed: {e}"

        self.capture_count += 1
        filename = f"capture_{self.capture_count:04d}.png"
        filepath = os.path.join(OUTPUT_DIR, filename)

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        cv2.imwrite(filepath, cv_image)

        t = transform.transform.translation
        q = transform.transform.rotation
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.capture_count, filename, stamp.sec, stamp.nanosec,
                t.x, t.y, t.z, q.x, q.y, q.z, q.w
            ])

        return True, f"Saved {filename} with pose."

    def auto_capture(self):
        success, message = self.do_capture()
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(f"Auto-capture skipped: {message}")

    def capture_callback(self, request, response):
        response.success, response.message = self.do_capture()
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()