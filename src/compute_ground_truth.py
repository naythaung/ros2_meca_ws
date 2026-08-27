import csv
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from scipy.spatial.transform import Rotation

CAPTURES_DIR = os.path.expanduser('~/meca_ws/captures')
INPUT_CSV = os.path.join(CAPTURES_DIR, 'poses.csv')
OUTPUT_CSV = os.path.join(CAPTURES_DIR, 'ground_truth_relative_to_phantom.csv')


class GroundTruthComputer(Node):
    def __init__(self):
        super().__init__('ground_truth_computer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.try_process)
        self.done = False

    def try_process(self):
        if self.done:
            return
        try:
            t = self.tf_buffer.lookup_transform('world', 'phantom', Time())
        except Exception as e:
            self.get_logger().warn(f"Waiting for world->phantom transform... ({e})")
            return

        t_wp = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        q_wp = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        R_wp = Rotation.from_quat(q_wp)

        if not os.path.exists(INPUT_CSV):
            self.get_logger().error(f"Input file not found: {INPUT_CSV}")
            self.done = True
            return

        with open(INPUT_CSV, 'r') as f_in, open(OUTPUT_CSV, 'w', newline='') as f_out:
            reader = csv.DictReader(f_in)
            writer = csv.writer(f_out)
            writer.writerow([
                'index', 'filename', 'stamp_sec', 'stamp_nanosec',
                'x_rel_phantom', 'y_rel_phantom', 'z_rel_phantom',
                'qx_rel_phantom', 'qy_rel_phantom', 'qz_rel_phantom', 'qw_rel_phantom',
                'range_m'
            ])

            count = 0
            for row in reader:
                t_wc = np.array([float(row['x']), float(row['y']), float(row['z'])])
                q_wc = [float(row['qx']), float(row['qy']), float(row['qz']), float(row['qw'])]
                R_wc = Rotation.from_quat(q_wc)

                # Compose: phantom -> camera = inverse(world->phantom) * (world->camera)
                t_pc = R_wp.inv().apply(t_wc - t_wp)
                R_pc = R_wp.inv() * R_wc
                q_pc = R_pc.as_quat()

                range_m = float(np.linalg.norm(t_pc))

                writer.writerow([
                    row['index'], row['filename'], row['stamp_sec'], row['stamp_nanosec'],
                    t_pc[0], t_pc[1], t_pc[2],
                    q_pc[0], q_pc[1], q_pc[2], q_pc[3],
                    range_m
                ])
                count += 1

        self.get_logger().info(f"Wrote {count} ground-truth poses to {OUTPUT_CSV}")
        self.done = True


def main():
    rclpy.init()
    node = GroundTruthComputer()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)


if __name__ == '__main__':
    main()