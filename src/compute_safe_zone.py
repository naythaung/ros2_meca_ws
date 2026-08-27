import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from visualization_msgs.msg import Marker

# --- your values ---
BOARD_WIDTH_M = 0.3        # breadboard width, in meters
BEHIND_LIMIT_M = 0.125     # measured: wall is 5 holes (5 x 25mm) behind the robot's base
FORWARD_MARGIN_M = 0.05    # extra clearance beyond the phantom
CEILING_HEIGHT_M = 0.210   # how high above the board surface the safe zone extends
Y_MARGIN_M = 0.02
# --------------------
# Note: BOARD_LENGTH_M isn't used anymore - the forward/backward (X) bounds now
# come directly from the phantom's real position and your measured wall distance,
# not the board's length, since that assumption didn't hold up earlier.


class SafeZoneComputer(Node):
    def __init__(self):
        super().__init__('safe_zone_computer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, 'safe_zone_marker', 10)
        self.timer = self.create_timer(0.5, self.compute)

    def compute(self):
        try:
            t_board = self.tf_buffer.lookup_transform('base_link', 'breadboard', Time())
            t_phantom = self.tf_buffer.lookup_transform('base_link', 'phantom', Time())
        except Exception as e:
            self.get_logger().warn(f"Waiting for transforms... ({e})")
            return

        board_z = t_board.transform.translation.z
        board_y = t_board.transform.translation.y
        phantom_x = t_phantom.transform.translation.x

        xmin = phantom_x - FORWARD_MARGIN_M
        xmax = BEHIND_LIMIT_M
        ymin = board_y - (BOARD_WIDTH_M / 2.0) - Y_MARGIN_M
        ymax = board_y + (BOARD_WIDTH_M / 2.0) + Y_MARGIN_M
        zmin = board_z - 0.005
        zmax = board_z + CEILING_HEIGHT_M

        self.get_logger().info(
            f"SetWorkZoneLimits({xmin*1000:.1f}, {ymin*1000:.1f}, {zmin*1000:.1f}, "
            f"{xmax*1000:.1f}, {ymax*1000:.1f}, {zmax*1000:.1f})",
            throttle_duration_sec=5.0
        )

        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'safe_zone'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = (xmin + xmax) / 2.0
        m.pose.position.y = (ymin + ymax) / 2.0
        m.pose.position.z = (zmin + zmax) / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = xmax - xmin
        m.scale.y = ymax - ymin
        m.scale.z = zmax - zmin
        m.color.r = 0.9
        m.color.g = 0.8
        m.color.b = 0.1
        m.color.a = 0.6  # bumped up so it's clearly visible for now
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = SafeZoneComputer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()