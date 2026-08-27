import os
import glob
import csv
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')  # headless - no GPU/display needed at all
import matplotlib.pyplot as plt

DATASET_DIR = os.path.expanduser('~/meca_ws/mock_dataset')
OUTPUT_DIR = os.path.join(DATASET_DIR, 'inspection')
MAX_POINTS_TO_PLOT = 4000  # subsample for speed/clarity in the 3D scatter

os.makedirs(OUTPUT_DIR, exist_ok=True)


def read_ply_points(path):
    points = []
    started = False
    with open(path) as f:
        for line in f:
            if started:
                x, y, z = map(float, line.split())
                points.append((x, y, z))
            if line.strip() == 'end_header':
                started = True
    return np.array(points)


def main():
    rgb_files = sorted(glob.glob(os.path.join(DATASET_DIR, 'rgb', '*_rgb.png')))
    if not rgb_files:
        print(f"No samples found in {DATASET_DIR}/rgb")
        return

    summary_path = os.path.join(OUTPUT_DIR, 'summary.csv')
    with open(summary_path, 'w', newline='') as summary_file:
        writer = csv.writer(summary_file)
        writer.writerow([
            'index', 'valid_pixels', 'total_pixels', 'valid_ratio',
            'depth_min_m', 'depth_max_m', 'depth_mean_m', 'num_points'
        ])

        for rgb_path in rgb_files:
            index = os.path.basename(rgb_path).split('_')[0]
            depth_path = os.path.join(DATASET_DIR, 'depth', f"{index}_depth.png")
            ply_path = os.path.join(DATASET_DIR, 'pointcloud', f"{index}_points.ply")

            if not (os.path.exists(depth_path) and os.path.exists(ply_path)):
                print(f"Skipping {index}: missing depth or pointcloud file")
                continue

            rgb = cv2.cvtColor(cv2.imread(rgb_path), cv2.COLOR_BGR2RGB)
            depth_mm = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            depth_m = depth_mm.astype(np.float32) / 1000.0
            valid_mask = depth_mm > 0

            points = read_ply_points(ply_path)

            n_valid = int(np.count_nonzero(valid_mask))
            n_total = depth_mm.size
            d_min = float(depth_m[valid_mask].min()) if n_valid > 0 else 0.0
            d_max = float(depth_m[valid_mask].max()) if n_valid > 0 else 0.0
            d_mean = float(depth_m[valid_mask].mean()) if n_valid > 0 else 0.0

            writer.writerow([
                index, n_valid, n_total, f"{n_valid/n_total:.3f}",
                f"{d_min:.4f}", f"{d_max:.4f}", f"{d_mean:.4f}", len(points)
            ])

            # --- Build the combined figure ---
            fig = plt.figure(figsize=(15, 5))

            ax1 = fig.add_subplot(1, 3, 1)
            ax1.imshow(rgb)
            ax1.set_title(f"Sample {index} - RGB")
            ax1.axis('off')

            ax2 = fig.add_subplot(1, 3, 2)
            depth_display = np.ma.masked_where(~valid_mask, depth_m)
            im = ax2.imshow(depth_display, cmap='viridis')
            ax2.set_title(f"Depth (m) - valid: {n_valid}/{n_total} ({100*n_valid/n_total:.0f}%)")
            ax2.axis('off')
            fig.colorbar(im, ax=ax2, fraction=0.046, pad=0.04, label='meters')

            ax3 = fig.add_subplot(1, 3, 3, projection='3d')
            if len(points) > 0:
                if len(points) > MAX_POINTS_TO_PLOT:
                    idx = np.random.choice(len(points), MAX_POINTS_TO_PLOT, replace=False)
                    plot_points = points[idx]
                else:
                    plot_points = points
                ax3.scatter(plot_points[:, 0], plot_points[:, 1], plot_points[:, 2],
                            s=1, c=plot_points[:, 2], cmap='viridis')
            ax3.set_title(f"World-frame points (n={len(points)})")
            ax3.set_xlabel('X (m)')
            ax3.set_ylabel('Y (m)')
            ax3.set_zlabel('Z (m)')

            plt.tight_layout()
            out_path = os.path.join(OUTPUT_DIR, f"{index}_inspection.png")
            plt.savefig(out_path, dpi=120)
            plt.close(fig)

            print(f"{index}: {n_valid}/{n_total} valid depth px ({100*n_valid/n_total:.0f}%), "
                  f"depth range [{d_min:.3f}, {d_max:.3f}]m, {len(points)} points -> {out_path}")

    print(f"\nDone. Summary written to {summary_path}")


if __name__ == '__main__':
    main()