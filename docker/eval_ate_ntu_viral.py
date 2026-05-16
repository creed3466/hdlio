#!/usr/bin/env python3
"""
eval_ate_ntu_viral.py — NTU VIRAL ATE (Absolute Trajectory Error) RMSE evaluator.

Computes ATE RMSE between SLAM trajectory CSV and NTU VIRAL ground truth CSV
using SE(3) alignment (Umeyama).

Features:
  - Lever arm correction: transforms body frame positions to prism frame
    using estimated orientation and known T_Body_Prism offset.
  - GT interpolation: linear interpolation instead of nearest-neighbor matching.

Usage:
  python3 eval_ate_ntu_viral.py <est_csv> <gt_csv> [options]

Input formats:
  est_csv: t_sec,tx,ty,tz,qx,qy,qz,qw  (SLAM output, body frame)
  gt_csv:  NTU VIRAL ground_truth.csv     (%time,...,pose.position.x,y,z,qx,qy,qz,qw)
"""

import argparse
import csv
import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d


# NTU VIRAL T_Body_Prism: translation from body/IMU frame to Leica prism
# Source: leica_prism.yaml (identical for all sequences)
T_BODY_PRISM_DEFAULT = np.array([-0.293656, -0.012288, -0.273095])


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
    """Load NTU VIRAL ground_truth.csv:
    %time,seq,stamp,pose.position.x,y,z,orientation.x,y,z,w"""
    data = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            if len(row) < 10:
                continue
            # stamp is in nanoseconds
            t = float(row[2]) * 1e-9  # nanoseconds -> seconds
            x, y, z = float(row[3]), float(row[4]), float(row[5])
            qx, qy, qz, qw = float(row[6]), float(row[7]), float(row[8]), float(row[9])
            data.append((t, x, y, z, qx, qy, qz, qw))
    return np.array(data) if data else None


def apply_lever_arm_correction(est, t_body_prism):
    """Transform body frame positions to prism positions.

    p_prism_world = R_world_body * t_body_prism + p_body_world

    The SLAM output gives body/IMU frame position and orientation.
    The GT gives Leica prism position. To compare them, we must transform
    the estimated body position to the prism position using the known
    lever arm and the estimated orientation.

    Args:
        est: Nx8 array [t, x, y, z, qx, qy, qz, qw]
        t_body_prism: [3,] lever arm vector in body frame (body -> prism)
    Returns:
        est_corrected: Nx8 array with corrected positions (prism frame)
    """
    est_corrected = est.copy()
    # scipy Rotation.from_quat expects [qx, qy, qz, qw] -- matches our format
    quats = est[:, 4:8]  # qx, qy, qz, qw
    R_wb = Rotation.from_quat(quats)
    # Rotate lever arm from body frame to world frame and add to body position
    prism_offset_world = R_wb.apply(t_body_prism)  # Nx3
    est_corrected[:, 1:4] += prism_offset_world
    return est_corrected


def associate_trajectories(est, gt, max_diff=0.05):
    """Associate estimated and ground truth trajectories by nearest timestamp.
    Returns matched indices (est_idx, gt_idx)."""
    est_times = est[:, 0]
    gt_times = gt[:, 0]

    est_indices = []
    gt_indices = []

    for i, et in enumerate(est_times):
        diffs = np.abs(gt_times - et)
        j = np.argmin(diffs)
        if diffs[j] < max_diff:
            est_indices.append(i)
            gt_indices.append(j)

    return np.array(est_indices), np.array(gt_indices)


def interpolate_gt_at_est_times(est, gt, gt_gap_threshold_s=0.5):
    """Interpolate GT positions at estimated trajectory timestamps.

    Uses linear interpolation instead of nearest-neighbor matching,
    eliminating ~25ms timing error from nearest-neighbor.

    Gap filtering: drops estimated poses whose nearest GT neighbor is
    farther than gt_gap_threshold_s seconds away. This prevents linear
    interpolation across Leica tracker dropout gaps (e.g. sbs_03 has
    a 15.37s dropout) from inflating ATE with fictitious GT positions.

    Args:
        est: Nx8 array [t, x, y, z, qx, qy, qz, qw]
        gt: Mx8 array [t, x, y, z, qx, qy, qz, qw]
        gt_gap_threshold_s: max allowed distance (seconds) to nearest GT
            sample. Frames farther than this are dropped. Default: 0.5s.
    Returns:
        est_matched: Kx8 subset of est within GT time range
        gt_interp: Kx4 array [t, x, y, z] interpolated GT positions
    """
    est_times = est[:, 0]
    gt_times = gt[:, 0]
    gt_positions = gt[:, 1:4]

    # Only interpolate within GT time range (no extrapolation)
    mask = (est_times >= gt_times[0]) & (est_times <= gt_times[-1])
    est_matched = est[mask]

    if len(est_matched) == 0:
        return None, None

    # GT gap filtering: remove est frames during GT dropout periods
    if gt_gap_threshold_s and gt_gap_threshold_s > 0:
        idx_next = np.searchsorted(gt_times, est_matched[:, 0], side='left')
        idx_prev = np.clip(idx_next - 1, 0, len(gt_times) - 1)
        idx_next = np.clip(idx_next, 0, len(gt_times) - 1)
        dist_prev = np.abs(est_matched[:, 0] - gt_times[idx_prev])
        dist_next = np.abs(est_matched[:, 0] - gt_times[idx_next])
        nearest = np.minimum(dist_prev, dist_next)
        keep = nearest <= gt_gap_threshold_s
        n_dropped = np.sum(~keep)
        if n_dropped > 0:
            print(f"GT gap filter: dropped {n_dropped}/{len(est_matched)} frames "
                  f"(gap > {gt_gap_threshold_s:.1f}s)")
        est_matched = est_matched[keep]
        if len(est_matched) == 0:
            return None, None

    # Linear interpolation of GT positions at est timestamps
    interp_func = interp1d(gt_times, gt_positions, axis=0, kind='linear')
    gt_pos_interp = interp_func(est_matched[:, 0])

    # Build gt_interp array: [t, x, y, z, 0, 0, 0, 1] (quaternion not needed for ATE)
    gt_interp = np.zeros((len(est_matched), 8))
    gt_interp[:, 0] = est_matched[:, 0]
    gt_interp[:, 1:4] = gt_pos_interp
    gt_interp[:, 7] = 1.0  # qw = 1 (placeholder)

    return est_matched, gt_interp


def umeyama_alignment(est_pos, gt_pos):
    """Compute SE(3) alignment (rotation + translation, no scale) using Umeyama method.
    Returns R, t such that gt ~= R @ est + t"""
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
    """Compute ATE RMSE after SE(3) alignment.

    Args:
        est_pos: Nx3 estimated positions
        gt_pos: Nx3 ground truth positions
    Returns:
        dict with rmse, mean, median, std, max, n_matches
    """
    # SE(3) alignment
    R, t = umeyama_alignment(est_pos, gt_pos)
    est_aligned = (R @ est_pos.T).T + t

    # ATE
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


def compute_ate(est, gt, est_idx, gt_idx):
    """Compute ATE RMSE after SE(3) alignment (legacy nearest-neighbor path)."""
    est_pos = est[est_idx, 1:4]
    gt_pos = gt[gt_idx, 1:4]

    result = compute_ate_from_matched(est_pos, gt_pos)
    result['n_est'] = len(est)
    result['n_gt'] = len(gt)
    return result


def main():
    parser = argparse.ArgumentParser(description='NTU VIRAL ATE RMSE Evaluator')
    parser.add_argument('est_csv', help='Estimated trajectory CSV (SLAM output)')
    parser.add_argument('gt_csv', help='Ground truth CSV (NTU VIRAL format)')
    parser.add_argument('--output_dir', default=None, help='Output directory for results')
    parser.add_argument('--max_diff', type=float, default=0.05,
                        help='Max timestamp difference for association (sec, nearest-neighbor mode)')

    # Lever arm correction
    lever_group = parser.add_mutually_exclusive_group()
    lever_group.add_argument('--lever_arm', action='store_true', default=True,
                             help='Enable lever arm correction (default: ON)')
    lever_group.add_argument('--no_lever_arm', action='store_true',
                             help='Disable lever arm correction')
    parser.add_argument('--t_body_prism', type=float, nargs=3, default=None,
                        metavar=('X', 'Y', 'Z'),
                        help='Override lever arm vector [x,y,z] in body frame '
                             '(default: [-0.293656, -0.012288, -0.273095])')

    # Interpolation
    interp_group = parser.add_mutually_exclusive_group()
    interp_group.add_argument('--interpolate', action='store_true', default=True,
                              help='Use linear interpolation for GT matching (default: ON)')
    interp_group.add_argument('--no_interpolate', action='store_true',
                              help='Use nearest-neighbor matching (legacy)')

    args = parser.parse_args()

    # Resolve flags
    use_lever_arm = not args.no_lever_arm
    use_interpolation = not args.no_interpolate
    t_body_prism = np.array(args.t_body_prism) if args.t_body_prism else T_BODY_PRISM_DEFAULT

    # Load trajectories
    est = load_est_traj(args.est_csv)
    if est is None or len(est) == 0:
        print(f"ERROR: No data in estimated trajectory: {args.est_csv}")
        sys.exit(1)

    gt = load_gt_traj(args.gt_csv)
    if gt is None or len(gt) == 0:
        print(f"ERROR: No data in ground truth: {args.gt_csv}")
        sys.exit(1)

    # Apply lever arm correction
    if use_lever_arm:
        est_eval = apply_lever_arm_correction(est, t_body_prism)
        print(f"Lever arm: ON  t_body_prism={t_body_prism}  "
              f"|t|={np.linalg.norm(t_body_prism):.4f}m")
    else:
        est_eval = est
        print("Lever arm: OFF")

    # Match trajectories
    if use_interpolation:
        est_matched, gt_matched = interpolate_gt_at_est_times(est_eval, gt)
        if est_matched is None or len(est_matched) < 10:
            print(f"ERROR: Only {0 if est_matched is None else len(est_matched)} "
                  f"matches within GT time range (need >= 10)")
            sys.exit(1)
        est_pos = est_matched[:, 1:4]
        gt_pos = gt_matched[:, 1:4]
        n_matches = len(est_matched)
        print(f"Matching:  interpolation (n={n_matches})")
    else:
        est_idx, gt_idx = associate_trajectories(est_eval, gt, args.max_diff)
        if len(est_idx) < 10:
            print(f"ERROR: Only {len(est_idx)} matches found (need >= 10)")
            sys.exit(1)
        est_pos = est_eval[est_idx, 1:4]
        gt_pos = gt[gt_idx, 1:4]
        n_matches = len(est_idx)
        print(f"Matching:  nearest-neighbor (max_diff={args.max_diff}s, n={n_matches})")

    # Compute ATE
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
            f.write(f"lever_arm: {use_lever_arm}\n")
            f.write(f"t_body_prism: {t_body_prism.tolist()}\n")
            f.write(f"interpolation: {use_interpolation}\n")
            for k, v in result.items():
                f.write(f"{k}: {v}\n")
        print(f"Results saved to: {result_file}")

    return result['rmse']


if __name__ == '__main__':
    rmse = main()
