#!/usr/bin/env python3
"""
eval_ate_m3dgr.py -- M3DGR ATE (Absolute Trajectory Error) RMSE evaluator.

Computes ATE RMSE between SLAM trajectory CSV and M3DGR ground truth CSV
using SE(3) alignment (Umeyama).

No lever arm correction (M3DGR GT is from VRPN/Optitrack, body frame).

Usage:
  python3 eval_ate_m3dgr.py <est_csv> <gt_csv> [options]

Input formats:
  est_csv: t_sec,tx,ty,tz,qx,qy,qz,qw  (SLAM output)
  gt_csv:  timestamp,x,y,z,qx,qy,qz,qw  (M3DGR GT, comma-separated or space-separated TUM)
"""

import argparse
import csv
import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d


def load_est_traj(path):
    """Load SLAM trajectory CSV: t_sec,tx,ty,tz,qx,qy,qz,qw"""
    data = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            if len(row) < 8:
                continue
            t = float(row[0])
            x, y, z = float(row[1]), float(row[2]), float(row[3])
            qx, qy, qz, qw = float(row[4]), float(row[5]), float(row[6]), float(row[7])
            data.append((t, x, y, z, qx, qy, qz, qw))
    return np.array(data) if data else None


def load_gt_traj(path):
    """Load M3DGR GT CSV: timestamp,x,y,z,qx,qy,qz,qw
    Also supports space-separated TUM format."""
    data = []
    with open(path, 'r') as f:
        first_line = f.readline().strip()
        # Detect format
        if ',' in first_line and not first_line.replace(',', '').replace('.', '').replace('-', '').replace('e', '').replace('+', '').isdigit():
            # CSV with header
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 8:
                    continue
                t = float(row[0])
                x, y, z = float(row[1]), float(row[2]), float(row[3])
                qx, qy, qz, qw = float(row[4]), float(row[5]), float(row[6]), float(row[7])
                data.append((t, x, y, z, qx, qy, qz, qw))
        else:
            # Space-separated TUM format (no header), process first line too
            def parse_line(line):
                parts = line.strip().split()
                if len(parts) >= 8:
                    return tuple(float(p) for p in parts[:8])
                return None

            result = parse_line(first_line)
            if result:
                data.append(result)
            for line in f:
                result = parse_line(line)
                if result:
                    data.append(result)

    return np.array(data) if data else None


def interpolate_gt_at_est_times(est, gt):
    """Interpolate GT positions at estimated trajectory timestamps."""
    est_times = est[:, 0]
    gt_times = gt[:, 0]
    gt_positions = gt[:, 1:4]

    mask = (est_times >= gt_times[0]) & (est_times <= gt_times[-1])
    est_matched = est[mask]

    if len(est_matched) == 0:
        return None, None

    interp_func = interp1d(gt_times, gt_positions, axis=0, kind='linear')
    gt_pos_interp = interp_func(est_matched[:, 0])

    gt_interp = np.zeros((len(est_matched), 8))
    gt_interp[:, 0] = est_matched[:, 0]
    gt_interp[:, 1:4] = gt_pos_interp
    gt_interp[:, 7] = 1.0

    return est_matched, gt_interp


def umeyama_alignment(est_pos, gt_pos):
    """Compute SE(3) alignment (rotation + translation, no scale) using Umeyama method."""
    assert est_pos.shape == gt_pos.shape
    n = est_pos.shape[0]

    mu_est = est_pos.mean(axis=0)
    mu_gt = gt_pos.mean(axis=0)

    est_centered = est_pos - mu_est
    gt_centered = gt_pos - mu_gt

    H = est_centered.T @ gt_centered / n

    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])

    R = Vt.T @ D @ U.T
    t = mu_gt - R @ mu_est

    return R, t


def compute_ate_from_matched(est_pos, gt_pos):
    """Compute ATE RMSE after SE(3) alignment."""
    R, t = umeyama_alignment(est_pos, gt_pos)
    est_aligned = (R @ est_pos.T).T + t

    errors = np.linalg.norm(est_aligned - gt_pos, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))
    mean_err = np.mean(errors)
    median_err = np.median(errors)
    std_err = np.std(errors)
    max_err = np.max(errors)

    return {
        'rmse': rmse,
        'mean': mean_err,
        'median': median_err,
        'std': std_err,
        'max': max_err,
        'n_matches': len(est_pos),
    }


def main():
    parser = argparse.ArgumentParser(description='M3DGR ATE RMSE Evaluator')
    parser.add_argument('est_csv', help='Estimated trajectory CSV (SLAM output)')
    parser.add_argument('gt_csv', help='Ground truth CSV (M3DGR format)')
    parser.add_argument('--output_dir', default=None, help='Output directory for results')

    args = parser.parse_args()

    est = load_est_traj(args.est_csv)
    if est is None or len(est) == 0:
        print(f"ERROR: No data in estimated trajectory: {args.est_csv}")
        sys.exit(1)

    gt = load_gt_traj(args.gt_csv)
    if gt is None or len(gt) == 0:
        print(f"ERROR: No data in ground truth: {args.gt_csv}")
        sys.exit(1)

    print(f"No lever arm correction (M3DGR: VRPN/Optitrack GT)")

    est_matched, gt_matched = interpolate_gt_at_est_times(est, gt)
    if est_matched is None or len(est_matched) < 10:
        print(f"ERROR: Only {0 if est_matched is None else len(est_matched)} "
              f"matches within GT time range (need >= 10)")
        print(f"Est time range: [{est[0,0]:.3f}, {est[-1,0]:.3f}]")
        print(f"GT time range:  [{gt[0,0]:.3f}, {gt[-1,0]:.3f}]")
        sys.exit(1)

    est_pos = est_matched[:, 1:4]
    gt_pos = gt_matched[:, 1:4]
    n_matches = len(est_matched)
    print(f"Matching: interpolation (n={n_matches})")

    result = compute_ate_from_matched(est_pos, gt_pos)
    result['n_est'] = len(est)
    result['n_gt'] = len(gt)
    result['n_matches'] = n_matches

    print(f"")
    print(f"ATE RMSE:    {result['rmse']:.4f} m")
    print(f"ATE Mean:    {result['mean']:.4f} m")
    print(f"ATE Median:  {result['median']:.4f} m")
    print(f"ATE Std:     {result['std']:.4f} m")
    print(f"ATE Max:     {result['max']:.4f} m")
    print(f"Matches:     {result['n_matches']} / est={result['n_est']} gt={result['n_gt']}")

    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)
        result_file = os.path.join(args.output_dir, 'ate_result.txt')
        with open(result_file, 'w') as f:
            for k, v in result.items():
                f.write(f"{k}: {v}\n")
        print(f"Results saved to: {result_file}")

    return result['rmse']


if __name__ == '__main__':
    rmse = main()
