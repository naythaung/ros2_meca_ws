import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from scipy.spatial.transform import Rotation


class AimChecker(Node):
    def __init__(self):
        super().__init__('aim_checker')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.check)

    def check(self):
        try:
            cam = self.tf_buffer.lookup_transform('world', 'camera_optical_frame', Time())
            phantom = self.tf_buffer.lookup_transform('world', 'phantom', Time())
        except Exception as e:
            self.get_logger().warn(f"Waiting for transforms... ({e})")
            return

        cam_pos = np.array([cam.transform.translation.x, cam.transform.translation.y, cam.transform.translation.z])
        phantom_pos = np.array([phantom.transform.translation.x, phantom.transform.translation.y, phantom.transform.translation.z])

        cam_quat = [cam.transform.rotation.x, cam.transform.rotation.y, cam.transform.rotation.z, cam.transform.rotation.w]
        cam_forward = Rotation.from_quat(cam_quat).apply([0, 0, 1])  # optical Z axis in world coords

        to_phantom = phantom_pos - cam_pos
        to_phantom_normalized = to_phantom / np.linalg.norm(to_phantom)

        angle_deg = np.degrees(np.arccos(np.clip(np.dot(cam_forward, to_phantom_normalized), -1, 1)))

        self.get_logger().info(
            f"Camera forward vs. direction-to-phantom: {angle_deg:.1f} degrees off "
            f"(0 = perfectly aimed, 180 = pointing exactly away)"
        )


def main():
    rclpy.init()
    rclpy.spin(AimChecker())


if __name__ == '__main__':
    main()