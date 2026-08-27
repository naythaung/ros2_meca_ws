import os
import csv
import time
import numpy as np
import cv2
import trimesh
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros
from scipy.spatial.transform import Rotation

# --- MOCK camera intrinsics - PLACEHOLDER until real calibration is done ---
IMG_WIDTH = 160
IMG_HEIGHT = 120
FX = 600.0
FY = 600.0
CX = IMG_WIDTH / 2.0
CY = IMG_HEIGHT / 2.0
# ----------------------------------------------------------------------

PHANTOM_MESH_PATH = os.path.expanduser(
    '~/meca_ws/src/meca500_scene_description/meshes/phantom.stl'
)
OUTPUT_DIR = os.path.expanduser('~/meca_ws/mock_dataset')
WORLD_FRAME = 'world'
PHANTOM_FRAME = 'phantom'
CAMERA_FRAME = 'camera_optical_frame'
CAPTURE_INTERVAL_S = 2.0
MAX_SAMPLES = 50


def write_ply(filepath, points):
    with open(filepath, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


class MockGroundTruthGenerator(Node):
    def __init__(self):
        super().__init__('mock_ground_truth_generator')

        for sub in ['rgb', 'depth', 'pointcloud', 'xyz_npy']:
            os.makedirs(os.path.join(OUTPUT_DIR, sub), exist_ok=True)

        self.csv_path = os.path.join(OUTPUT_DIR, 'poses.csv')
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'index', 'rgb_file', 'depth_file', 'pointcloud_file', 'xyz_npy_file',
                    'stamp_sec', 'stamp_nanosec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
                ])

        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.count = self._count_existing_samples()
        self.mesh_ready = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Precompute one ray direction per pixel, in the camera's own local frame
        us, vs = np.meshgrid(np.arange(IMG_WIDTH), np.arange(IMG_HEIGHT))
        xs = (us - CX) / FX
        ys = (vs - CY) / FY
        zs = np.ones_like(xs)
        dirs = np.stack([xs, ys, zs], axis=-1).reshape(-1, 3)
        self.local_ray_dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)

        self.create_subscription(Image, 'image_raw', self.image_cb, 10)
        self.create_timer(1.0, self.try_setup_mesh)
        self.create_timer(CAPTURE_INTERVAL_S, self.try_generate)

        self.get_logger().info("Waiting for world->phantom transform to place mesh in world frame...")

    def _count_existing_samples(self):
        rgb_dir = os.path.join(OUTPUT_DIR, 'rgb')
        if not os.path.exists(rgb_dir):
            return 0
        existing = [f for f in os.listdir(rgb_dir) if f.endswith('_rgb.png')]
        return len(existing)
    
    def try_setup_mesh(self):
        if self.mesh_ready:
            return
        try:
            t = self.tf_buffer.lookup_transform(WORLD_FRAME, PHANTOM_FRAME, Time())
        except Exception as e:
            self.get_logger().warn(f"Waiting for world->phantom transform... ({e})")
            return

        pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        R = Rotation.from_quat(quat)

        T = np.eye(4)
        T[:3, :3] = R.as_matrix()
        T[:3, 3] = pos

        self.get_logger().info(f"Loading phantom mesh from {PHANTOM_MESH_PATH} ...")
        self.mesh = trimesh.load(PHANTOM_MESH_PATH)
        self.mesh.apply_scale(0.001)   # STL authored in mm -> meters
        self.mesh.apply_transform(T)   # now permanently expressed in WORLD coordinates
        self.get_logger().info(f"Mesh loaded and placed in world frame: {len(self.mesh.faces)} triangles.")
        self.mesh_ready = True

    def image_cb(self, msg):
        self.latest_image_msg = msg

    def try_generate(self):
        if not self.mesh_ready:
            return
        if self.latest_image_msg is None:
            self.get_logger().warn("No image received yet - waiting for /image_raw", throttle_duration_sec=5.0)
            return

        img_msg = self.latest_image_msg
        stamp = img_msg.header.stamp

        try:
            t = self.tf_buffer.lookup_transform(
                WORLD_FRAME, CAMERA_FRAME, Time(),  # latest available - avoids extrapolation errors during bag playback
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed, skipping: {e}")
            return

        cam_pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        R = Rotation.from_quat(quat)

        ray_origins = np.tile(cam_pos, (self.local_ray_dirs.shape[0], 1))
        ray_dirs_world = R.apply(self.local_ray_dirs)  # rays now cast directly in world frame

        self.get_logger().info(f"Casting {self.local_ray_dirs.shape[0]} rays...")
        t_start = time.time()
        locations, index_ray, _ = self.mesh.ray.intersects_location(
            ray_origins=ray_origins, ray_directions=ray_dirs_world, multiple_hits=False
        )
        raycast_ms = (time.time() - t_start) * 1000.0

        # Full per-pixel world-frame XYZ map, NaN where no ray hit
        xyz_map = np.full((self.local_ray_dirs.shape[0], 3), np.nan, dtype=np.float32)
        if len(locations) > 0:
            xyz_map[index_ray] = locations.astype(np.float32)

        # Camera-frame depth (Z-axis only), kept for simple depth-map-style evaluation too
        depth_flat = np.zeros(self.local_ray_dirs.shape[0], dtype=np.float32)
        if len(locations) > 0:
            hits_camera_frame = R.inv().apply(locations - cam_pos)
            depth_flat[index_ray] = hits_camera_frame[:, 2]

        self.count += 1
        rgb_filename = f"{self.count:04d}_rgb.png"
        depth_filename = f"{self.count:04d}_depth.png"
        ply_filename = f"{self.count:04d}_points.ply"
        npy_filename = f"{self.count:04d}_xyz.npy"

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        cv2.imwrite(os.path.join(OUTPUT_DIR, 'rgb', rgb_filename), cv_image)

        depth_mm = np.clip(depth_flat.reshape(IMG_HEIGHT, IMG_WIDTH) * 1000.0, 0, 65535).astype(np.uint16)
        cv2.imwrite(os.path.join(OUTPUT_DIR, 'depth', depth_filename), depth_mm)

        try:
            valid_points = xyz_map[~np.isnan(xyz_map).any(axis=1)]
            write_ply(os.path.join(OUTPUT_DIR, 'pointcloud', ply_filename), valid_points)

            np.save(os.path.join(OUTPUT_DIR, 'xyz_npy', npy_filename), xyz_map.reshape(IMG_HEIGHT, IMG_WIDTH, 3))
        except Exception as e:
            self.get_logger().error(f"Error saving sample {self.count}: {e}")
            
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                self.count, rgb_filename, depth_filename, ply_filename, npy_filename,
                stamp.sec, stamp.nanosec,
                cam_pos[0], cam_pos[1], cam_pos[2],
                quat[0], quat[1], quat[2], quat[3]
            ])

        self.get_logger().info(
            f"Saved sample {self.count} - {len(valid_points)}/{depth_flat.size} valid points "
            f"(ray-cast took {raycast_ms:.0f}ms)"
        )

        if self.count >= MAX_SAMPLES:
            self.get_logger().info(f"Reached {MAX_SAMPLES} samples - shutting down.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MockGroundTruthGenerator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()