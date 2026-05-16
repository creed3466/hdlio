#!/usr/bin/env python3
"""
eval_baselines_ate.py — Compute ATE RMSE for baseline algorithm trajectories.

Handles both comma-separated and space-separated TUM format.
Uses the same Umeyama SE(3) alignment as eval_ate_m3dgr.py.

Usage:
  python3 eval_baselines_ate.py <dump_root> <gt_dir>

Example:
  python3 eval_baselines_ate.py dump/indoor_baselines /home/euntae/Project/dataset/ros1/surfel_data/ground_truth
"""

import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d


def load_traj(path):
    """Load trajectory from TUM format (space or comma separated)."""
    data = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            # Try comma first, then space
            if ',' in line:
                parts = line.split(',')
            else:
                parts = line.split()
            if len(parts) >= 8:
                try:
                    data.append([float(x) for x in parts[:8]])
                except ValueError:
                    continue
    return np.array(data) if data else None


def load_gt(path):
    """Load M3DGR GT (space-separated TUM)."""
    return load_traj(path)


def umeyama_alignment(est_xyz, gt_xyz):
    """SE(3) Umeyama alignment (rotation + translation, no scale)."""
    n = est_xyz.shape[0]
    mu_est = est_xyz.mean(axis=0)
    mu_gt = gt_xyz.mean(axis=0)
    est_c = est_xyz - mu_est
    gt_c = gt_xyz - mu_gt
    H = est_c.T @ gt_c
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    t = mu_gt - R @ mu_est
    return R, t


def compute_ate(est_path, gt_path):
    """Compute ATE RMSE with SE(3) alignment."""
    est = load_traj(est_path)
    gt = load_gt(gt_path)

    if est is None or len(est) < 10:
        return None, "no_est_data"
    if gt is None or len(gt) < 10:
        return None, "no_gt_data"

    # Interpolate GT to est timestamps
    gt_times = gt[:, 0]
    est_times = est[:, 0]

    # Clip to overlapping time range
    t_min = max(gt_times[0], est_times[0])
    t_max = min(gt_times[-1], est_times[-1])

    mask = (est_times >= t_min) & (est_times <= t_max)
    est_valid = est[mask]

    if len(est_valid) < 10:
        return None, "insufficient_overlap"

    # Interpolate GT xyz
    gt_interp = np.zeros((len(est_valid), 3))
    for i in range(3):
        f = interp1d(gt_times, gt[:, 1 + i], kind='linear', fill_value='extrapolate')
        gt_interp[:, i] = f(est_valid[:, 0])

    est_xyz = est_valid[:, 1:4]

    # Umeyama alignment
    R, t = umeyama_alignment(est_xyz, gt_interp)
    est_aligned = (R @ est_xyz.T).T + t

    # ATE
    errors = np.linalg.norm(est_aligned - gt_interp, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))
    mean_err = np.mean(errors)
    std_err = np.std(errors)
    max_err = np.max(errors)

    return {
        'rmse': rmse,
        'mean': mean_err,
        'std': std_err,
        'max': max_err,
        'n_matches': len(est_valid),
        'n_est': len(est),
        'n_gt': len(gt),
    }, None


def main():
    dump_root = sys.argv[1] if len(sys.argv) > 1 else "dump/indoor_baselines"
    gt_dir = sys.argv[2] if len(sys.argv) > 2 else "/home/euntae/Project/dataset/ros1/surfel_data/ground_truth"

    algos = ['fast_lio2', 'ig_lio', 'point_lio']
    datasets = ['avia', 'mid360']
    seqs = [
        'indoor_Dark03', 'indoor_Dark04',
        'indoor_Dynamic01', 'indoor_Dynamic02',
        'indoor_Occlusion01', 'indoor_Occlusion02',
        'indoor_Varying-illu01', 'indoor_Varying-illu02',
    ]

    for algo in algos:
        for dataset in datasets:
            print(f"\n=== {algo} / {dataset} ===")
            total = 0
            count = 0
            for seq in seqs:
                traj_path = os.path.join(dump_root, algo, dataset, seq, 'traj.csv')
                gt_path = os.path.join(gt_dir, f'{seq}.txt')
                ate_out = os.path.join(dump_root, algo, dataset, seq, 'ate_result.txt')

                if not os.path.exists(traj_path):
                    print(f"  {seq:25s} MISSING")
                    continue

                result, err = compute_ate(traj_path, gt_path)
                if result is None:
                    print(f"  {seq:25s} FAIL ({err})")
                    continue

                rmse = result['rmse']
                print(f"  {seq:25s} {rmse:.6f} m")
                total += rmse
                count += 1

                # Write ate_result.txt
                with open(ate_out, 'w') as f:
                    for k, v in result.items():
                        f.write(f"{k}: {v}\n")

            if count > 0:
                mean = total / count
                print(f"  Mean ({count}/{len(seqs)}): {mean:.6f} m")


if __name__ == '__main__':
    main()
