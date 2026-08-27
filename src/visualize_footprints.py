import csv
import os
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation

CAPTURES_CSV = os.path.expanduser('~/meca_ws/captures/poses.csv')
PLANE_SIZE = 0.03      # 3cm square - placeholder, not real FOV until calibrated
FORWARD_OFFSET = 0.02  # sits just in front of the lens, along viewing direction


class FootprintPublisher(Node):
    def __init__(self):
        super().__init__('capture_footprint_viz')
        self.pub = self.create_publisher(MarkerArray, 'capture_footprints', 10)
        self.rows = self._load_rows()
        self.timer = self.create_timer(1.0, self.publish_markers)

    def _load_rows(self):
        if not os.path.exists(CAPTURES_CSV):
            self.get_logger().error(f"Not found: {CAPTURES_CSV}")
            return []
        with open(CAPTURES_CSV) as f:
            return list(csv.DictReader(f))

    def publish_markers(self):
        arr = MarkerArray()
        for i, row in enumerate(self.rows):
            pos = np.array([float(row['x']), float(row['y']), float(row['z'])])
            quat = [float(row['qx']), float(row['qy']), float(row['qz']), float(row['qw'])]
            forward = Rotation.from_quat(quat).apply([0, 0, 1])
            plane_center = pos + forward * FORWARD_OFFSET

            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'capture_footprints'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = plane_center
            m.pose.orientation.x = quat[0]
            m.pose.orientation.y = quat[1]
            m.pose.orientation.z = quat[2]
            m.pose.orientation.w = quat[3]
            m.scale.x = PLANE_SIZE
            m.scale.y = PLANE_SIZE
            m.scale.z = 0.001
            m.color.r = 0.2
            m.color.g = 0.6
            m.color.b = 0.9
            m.color.a = 0.5
            arr.markers.append(m)
        self.pub.publish(arr)
        self.get_logger().info(f"Published {len(arr.markers)} footprints.", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = FootprintPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()