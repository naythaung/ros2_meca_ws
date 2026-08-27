import os
import glob
import cv2
import numpy as np

DATASET_DIR = os.path.expanduser('~/meca_ws/mock_dataset')
RGB_DIR = os.path.join(DATASET_DIR, 'rgb')
DEPTH_DIR = os.path.join(DATASET_DIR, 'depth')
OUTPUT_DIR = os.path.join(DATASET_DIR, 'comparisons')

os.makedirs(OUTPUT_DIR, exist_ok=True)

depth_files = sorted(glob.glob(os.path.join(DEPTH_DIR, '*_depth.png')))

if not depth_files:
    print(f"No depth files found in {DEPTH_DIR}")

for depth_path in depth_files:
    basename = os.path.basename(depth_path)
    index = basename.split('_')[0]
    rgb_path = os.path.join(RGB_DIR, f"{index}_rgb.png")

    if not os.path.exists(rgb_path):
        print(f"Skipping {index}: no matching RGB file")
        continue

    depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    rgb_img = cv2.imread(rgb_path, cv2.IMREAD_COLOR)

    if depth_img is None or rgb_img is None:
        print(f"Skipping {index}: failed to load image(s)")
        continue

    mask = depth_img > 0
    n_valid = int(np.count_nonzero(mask))

    stretched = np.zeros(depth_img.shape, dtype=np.uint8)
    if mask.any():
        normalized = cv2.normalize(depth_img[mask], None, 50, 255, cv2.NORM_MINMAX).astype(np.uint8).flatten()
        stretched[mask] = normalized

    depth_color = cv2.applyColorMap(stretched, cv2.COLORMAP_JET)
    depth_color[~mask] = (30, 30, 30)  # dark grey = no hit / background

    # Depth may be lower resolution than RGB (e.g. our 80x60 test run vs 640x480 RGB) - resize to match
    if depth_color.shape[:2] != rgb_img.shape[:2]:
        depth_color = cv2.resize(
            depth_color, (rgb_img.shape[1], rgb_img.shape[0]),
            interpolation=cv2.INTER_NEAREST  # nearest-neighbor: keeps it honest about actual resolution, no fake smoothing
        )

    combined = np.hstack([rgb_img, depth_color])
    out_path = os.path.join(OUTPUT_DIR, f"{index}_comparison.png")
    cv2.imwrite(out_path, combined)
    print(f"{index}: {n_valid}/{depth_img.size} valid depth pixels -> {out_path}")

print(f"\nDone. Comparisons saved to {OUTPUT_DIR}")