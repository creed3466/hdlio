#!/usr/bin/env python3
"""
eval_ate_gt_aware.py — GT-gap-aware ATE evaluator for NTU VIRAL.

Computes ATE RMSE excluding points where the ground truth has large temporal gaps,
which makes the interpolated GT positions unreliable.

Algorithm:
  1. Load GT and estimated trajectories
  2. Apply lever arm correction (same as eval_ate_ntu_viral.py)
  3. For each estimated point, find the GT gap (time between the two nearest GT
     timestamps bracketing it)
  4. Compute SE(3) alignment on ALL points (same alignment as standard eval)
  5. Compute ATE for subsets filtered by GT gap threshold

Usage:
  python3 eval_ate_gt_aware.py <est_csv> <gt_csv> [--label LABEL]
"""

import argparse
import csv
import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d


# NTU VIRAL T_Body_Prism
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
    """Load NTU VIRAL ground_truth.csv"""
    data = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            if len(row) < 10:
                continue
            t = float(row[2]) * 1e-9  # nanoseconds -> seconds
            x, y, z = float(row[3]), float(row[4]), float(row[5])
            qx, qy, qz, qw = float(row[6]), float(row[7]), float(row[8]), float(row[9])
            data.append((t, x, y, z, qx, qy, qz, qw))
    return np.array(data) if data else None


def apply_lever_arm_correction(est, t_body_prism):
    """Transform body frame positions to prism positions."""
    est_corrected = est.copy()
    quats = est[:, 4:8]
    R_wb = Rotation.from_quat(quats)
    prism_offset_world = R_wb.apply(t_body_prism)
    est_corrected[:, 1:4] += prism_offset_world
    return est_corrected


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


def compute_gt_gaps(est_times, gt_times):
    """For each estimated timestamp, compute the GT gap at that point.

    The GT gap is the time between the two nearest GT timestamps that
    bracket the estimated timestamp. If the est timestamp exactly matches
    a GT timestamp, the gap is 0.

    Returns:
        gaps: array of GT gap durations (seconds) for each est timestamp
    """
    gt_sorted = np.sort(gt_times)
    gaps = np.zeros(len(est_times))

    # Use searchsorted to find bracketing GT timestamps efficiently
    indices = np.searchsorted(gt_sorted, est_times)

    for i, (et, idx) in enumerate(zip(est_times, indices)):
        if idx == 0:
            # Before first GT point - gap is 0 (will be filtered by time range anyway)
            gaps[i] = 0.0
        elif idx >= len(gt_sorted):
            # After last GT point
            gaps[i] = 0.0
        else:
            t_before = gt_sorted[idx - 1]
            t_after = gt_sorted[idx]
            gaps[i] = t_after - t_before

    return gaps


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


def main():
    parser = argparse.ArgumentParser(description='GT-gap-aware ATE Evaluator')
    parser.add_argument('est_csv', help='Estimated trajectory CSV')
    parser.add_argument('gt_csv', help='Ground truth CSV (NTU VIRAL format)')
    parser.add_argument('--label', default='', help='Label for output')
    parser.add_argument('--thresholds', type=float, nargs='+',
                        default=[5.0, 2.0, 1.0, 0.5],
                        help='GT gap thresholds in seconds')
    args = parser.parse_args()

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
    t_body_prism = T_BODY_PRISM_DEFAULT
    est_eval = apply_lever_arm_correction(est, t_body_prism)
    print(f"Lever arm: ON  t_body_prism={t_body_prism}  |t|={np.linalg.norm(t_body_prism):.4f}m")

    # Interpolate GT at est times
    est_matched, gt_matched = interpolate_gt_at_est_times(est_eval, gt)
    if est_matched is None or len(est_matched) < 10:
        print(f"ERROR: Insufficient matches")
        sys.exit(1)

    est_pos = est_matched[:, 1:4]
    gt_pos = gt_matched[:, 1:4]
    n_total = len(est_pos)
    print(f"Matching:  interpolation (n={n_total})")

    # Compute GT gaps for each matched point
    gt_gaps = compute_gt_gaps(est_matched[:, 0], gt[:, 0])

    # GT gap statistics
    print(f"\nGT gap statistics:")
    print(f"  Min gap:    {gt_gaps.min():.4f}s")
    print(f"  Max gap:    {gt_gaps.max():.4f}s")
    print(f"  Mean gap:   {gt_gaps.mean():.4f}s")
    print(f"  Median gap: {np.median(gt_gaps):.4f}s")
    for thresh in [0.5, 1.0, 2.0, 5.0, 10.0, 15.0]:
        n_above = np.sum(gt_gaps > thresh)
        print(f"  Points with gap > {thresh:5.1f}s: {n_above:5d} ({100.0*n_above/n_total:.1f}%)")

    # SE(3) alignment on ALL points (IMPORTANT: same alignment for all thresholds)
    R, t = umeyama_alignment(est_pos, gt_pos)
    est_aligned = (R @ est_pos.T).T + t

    # Per-point errors
    errors = np.linalg.norm(est_aligned - gt_pos, axis=1)

    # Print results
    label = args.label if args.label else os.path.basename(args.est_csv)
    print(f"\n{'='*60}")
    print(f"{label} - GT-gap-aware ATE:")
    print(f"{'='*60}")

    # All points
    rmse_all = np.sqrt(np.mean(errors ** 2))
    print(f"  All points:        RMSE={rmse_all:.4f}m ({n_total} points)")

    # Filtered by threshold
    for thresh in sorted(args.thresholds, reverse=True):
        mask = gt_gaps < thresh
        n_reliable = np.sum(mask)
        n_excluded = n_total - n_reliable
        pct_excluded = 100.0 * n_excluded / n_total
        if n_reliable > 0:
            rmse_filtered = np.sqrt(np.mean(errors[mask] ** 2))
            print(f"  Gap < {thresh:.1f}s only:   RMSE={rmse_filtered:.4f}m "
                  f"({n_reliable} points, {pct_excluded:.1f}% excluded)")
        else:
            print(f"  Gap < {thresh:.1f}s only:   No reliable points!")

    # Also show the errors at the excluded points (for insight)
    print(f"\n  Error analysis at large-gap points (gap >= 5.0s):")
    large_gap_mask = gt_gaps >= 5.0
    if np.any(large_gap_mask):
        large_gap_errors = errors[large_gap_mask]
        print(f"    N points:  {np.sum(large_gap_mask)}")
        print(f"    Mean err:  {np.mean(large_gap_errors):.4f}m")
        print(f"    Max err:   {np.max(large_gap_errors):.4f}m")
        print(f"    RMSE:      {np.sqrt(np.mean(large_gap_errors**2)):.4f}m")
    else:
        print(f"    No points with gap >= 5.0s")

    print()
    return rmse_all


if __name__ == '__main__':
    main()
