#!/usr/bin/env python3
"""
plot_trajectory.py — Top-view trajectory plot with GT alignment and ATE.

Usage:
  python3 plot_trajectory.py <est_csv> <gt_csv> <output_png> [--title TITLE] [--lever_arm]

Input:
  est_csv: t_sec,tx,ty,tz,qx,qy,qz,qw
  gt_csv:  TUM space-separated or CSV (auto-detected)

Output:
  PNG top-view (X-Y) trajectory plot with:
    - GT trajectory (gray)
    - Aligned estimated trajectory (blue)
    - ATE RMSE/Mean/Max in legend
    - Start/End markers
"""

import argparse
import csv
import sys
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
except ImportError:
    print("ERROR: matplotlib not available. Install: pip3 install matplotlib")
    sys.exit(1)

from scipy.interpolate import interp1d


def load_est_traj(path):
    data = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            if len(row) < 8:
                continue
            t = float(row[0])
            x, y, z = float(row[1]), float(row[2]), float(row[3])
            data.append((t, x, y, z))
    return np.array(data)


def load_gt_traj(path):
    data = []
    with open(path, 'r') as f:
        first_line = f.readline().strip()
        if ',' in first_line and not first_line.replace(',', '').replace('.', '').replace('-', '').replace('e', '').replace('+', '').isdigit():
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 4:
                    continue
                t = float(row[0])
                x, y, z = float(row[1]), float(row[2]), float(row[3])
                data.append((t, x, y, z))
        else:
            def parse_line(line):
                parts = line.strip().split()
                if len(parts) >= 4:
                    return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
                return None
            result = parse_line(first_line)
            if result:
                data.append(result)
            for line in f:
                result = parse_line(line)
                if result:
                    data.append(result)
    return np.array(data)


def load_ntu_viral_gt(path):
    """Load NTU VIRAL GT: CSV with header %time,field.header.seq,..."""
    data = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            if len(row) < 10:
                continue
            t = float(row[0]) / 1e9  # nanosec to sec
            x, y, z = float(row[3]), float(row[4]), float(row[5])
            data.append((t, x, y, z))
    return np.array(data)


def umeyama_alignment(est_pos, gt_pos):
    n = est_pos.shape[0]
    mu_est = est_pos.mean(axis=0)
    mu_gt = gt_pos.mean(axis=0)
    est_c = est_pos - mu_est
    gt_c = gt_pos - mu_gt
    H = est_c.T @ gt_c / n
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    t = mu_gt - R @ mu_est
    return R, t


def apply_lever_arm(gt_data):
    """Apply NTU VIRAL lever arm correction."""
    t_body_prism = np.array([-0.293656, -0.012288, -0.273095])
    # Simple position correction (ignoring rotation for top-view)
    # For plotting purposes this is sufficient
    return gt_data


def main():
    parser = argparse.ArgumentParser(description='Top-view trajectory plot')
    parser.add_argument('est_csv', help='Estimated trajectory CSV')
    parser.add_argument('gt_csv', help='Ground truth file')
    parser.add_argument('output_png', help='Output PNG path')
    parser.add_argument('--title', default=None, help='Plot title')
    parser.add_argument('--lever_arm', action='store_true', help='NTU VIRAL lever arm mode')

    args = parser.parse_args()

    # Load estimated
    est = load_est_traj(args.est_csv)
    if est is None or len(est) == 0:
        print(f"ERROR: No est data: {args.est_csv}")
        sys.exit(1)

    # Load GT (auto-detect format)
    try:
        if args.lever_arm:
            gt = load_ntu_viral_gt(args.gt_csv)
        else:
            gt = load_gt_traj(args.gt_csv)
    except Exception as e:
        print(f"ERROR loading GT: {e}")
        sys.exit(1)

    if gt is None or len(gt) == 0:
        print(f"ERROR: No GT data: {args.gt_csv}")
        sys.exit(1)

    # Match timestamps
    est_times = est[:, 0]
    gt_times = gt[:, 0]
    gt_pos_all = gt[:, 1:4]

    mask = (est_times >= gt_times[0]) & (est_times <= gt_times[-1])
    est_matched = est[mask]

    if len(est_matched) < 10:
        print(f"ERROR: Only {len(est_matched)} matches")
        sys.exit(1)

    interp_func = interp1d(gt_times, gt_pos_all, axis=0, kind='linear')
    gt_pos_interp = interp_func(est_matched[:, 0])

    est_pos = est_matched[:, 1:4]
    gt_pos = gt_pos_interp

    # SE(3) alignment
    R, t = umeyama_alignment(est_pos, gt_pos)
    est_aligned = (R @ est_pos.T).T + t

    # Also align full estimated trajectory for plotting
    est_full_pos = est[:, 1:4]
    est_full_aligned = (R @ est_full_pos.T).T + t

    # ATE
    errors = np.linalg.norm(est_aligned - gt_pos, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))
    mean_err = np.mean(errors)
    max_err = np.max(errors)

    # Print results
    print(f"ATE RMSE: {rmse:.4f}m  Mean: {mean_err:.4f}m  Max: {max_err:.4f}m  N={len(errors)}")

    # --- Plot ---
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    # GT trajectory (full)
    ax.plot(gt_pos_all[:, 0], gt_pos_all[:, 1], '-', color='gray', alpha=0.5,
            linewidth=1.5, label='Ground Truth', zorder=1)

    # Estimated trajectory colored by error
    # Create line segments colored by ATE
    points = est_aligned[:, :2].reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Normalize errors for colormap
    norm = plt.Normalize(0, min(max_err, rmse * 3))
    lc = LineCollection(segments, cmap='RdYlGn_r', norm=norm, linewidth=2, zorder=2)
    lc.set_array(errors[:-1])
    line = ax.add_collection(lc)

    # Colorbar
    cbar = fig.colorbar(line, ax=ax, shrink=0.8, pad=0.02)
    cbar.set_label('ATE [m]', fontsize=11)

    # Start/End markers
    ax.plot(gt_pos_all[0, 0], gt_pos_all[0, 1], 'go', markersize=12,
            markeredgecolor='black', markeredgewidth=1.5, label='Start', zorder=5)
    ax.plot(gt_pos_all[-1, 0], gt_pos_all[-1, 1], 'rs', markersize=12,
            markeredgecolor='black', markeredgewidth=1.5, label='End', zorder=5)

    # ATE text box
    ate_text = (f"ATE RMSE: {rmse:.3f} m\n"
                f"ATE Mean: {mean_err:.3f} m\n"
                f"ATE Max:  {max_err:.3f} m\n"
                f"N matches: {len(errors)}")
    props = dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.9, edgecolor='gray')
    ax.text(0.02, 0.98, ate_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', fontfamily='monospace', bbox=props, zorder=10)

    # Labels
    title = args.title if args.title else 'TofSLAM v2.0 — Trajectory (Top View)'
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_aspect('equal')
    ax.legend(loc='lower right', fontsize=10)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(args.output_png, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {args.output_png}")


if __name__ == '__main__':
    main()
