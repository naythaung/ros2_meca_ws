import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from scipy.spatial.transform import Rotation

WORLD_FRAME = 'world'
CAMERA_FRAME = 'camera_optical_frame'
PLANE_SIZE = 0.03
FORWARD_OFFSET = 0.02
TRAIL_MIN_DIST_M = 0.005  # only drop a new trail mark once moved this far
PUBLISH_RATE_HZ = 10.0


class LiveFootprintViz(Node):
    def __init__(self):
        super().__init__('live_footprint_viz')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.trail_pub = self.create_publisher(MarkerArray, 'capture_footprint_trail', 10)
        self.current_pub = self.create_publisher(Marker, 'capture_footprint_current', 10)
        self.trail_markers = []
        self.last_pos = None
        self.next_id = 0
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.update)

    def update(self):
        try:
            t = self.tf_buffer.lookup_transform(WORLD_FRAME, CAMERA_FRAME, Time())
        except Exception:
            return  # nothing playing yet / this frame not reached

        pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        forward = Rotation.from_quat(quat).apply([0, 0, 1])
        center = pos + forward * FORWARD_OFFSET

        cur = self._make_marker(center, quat, 0, 0.95, 0.6, 0.1, 0.85, 'current')
        self.current_pub.publish(cur)

        if self.last_pos is None or np.linalg.norm(pos - self.last_pos) > TRAIL_MIN_DIST_M:
            self.trail_markers.append(self._make_marker(center, quat, self.next_id, 0.2, 0.6, 0.9, 0.35, 'trail'))
            self.next_id += 1
            self.last_pos = pos

        arr = MarkerArray()
        arr.markers = self.trail_markers
        self.trail_pub.publish(arr)

    def _make_marker(self, center, quat, mid, r, g, b, a, ns):
        m = Marker()
        m.header.frame_id = WORLD_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = center
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat
        m.scale.x = m.scale.y = PLANE_SIZE
        m.scale.z = 0.001
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        return m


def main():
    rclpy.init()
    rclpy.spin(LiveFootprintViz())


if __name__ == '__main__':
    main()