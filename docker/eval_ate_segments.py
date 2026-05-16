#!/usr/bin/env python3
"""eval_ate_segments.py — per-segment ATE RMSE evaluator.

Splits an estimated trajectory into N equal-time segments (by estimated-traj
timestamp range) and computes ATE RMSE per segment with a single global
Umeyama SE(3) alignment, to diagnose whether error is uniformly distributed
(contamination-dominant) or late-heavy (loop-closure-absence-dominant).

Motivation (Task #71 Iter 2d §10.2):
    Decide whether VI03's 0.160 m gap above FAST-LIO2 SOTA 0.897 m is
    correspondence contamination (frontend, addressable by D2) or
    accumulated drift (backend, needs loop closure).

Usage:
    python3 eval_ate_segments.py <est_csv> <gt_txt> [--segments 5]

Input:
    est_csv: t_sec,tx,ty,tz,qx,qy,qz,qw
    gt_txt:  TUM space-separated: t x y z qx qy qz qw
"""
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class SegmentResult:
    idx: int
    t_start: float
    t_end: float
    n_matches: int
    rmse: float
    mean: float
    max: float


def load_est(path: str) -> np.ndarray:
    rows: list[list[float]] = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        next(reader)  # header
        for row in reader:
            if len(row) < 8:
                continue
            rows.append([float(v) for v in row[:8]])
    if not rows:
        raise ValueError(f"empty est traj: {path}")
    return np.asarray(rows, dtype=float)


def load_gt_tum(path: str) -> np.ndarray:
    rows: list[list[float]] = []
    with open(path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 8:
                rows.append([float(v) for v in parts[:8]])
    if not rows:
        raise ValueError(f"empty GT: {path}")
    return np.asarray(rows, dtype=float)


def interpolate_gt(est: np.ndarray, gt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (est_matched, gt_pos_interp) within GT time range.

    Linear interpolation per axis using numpy (scipy-free).
    """
    gt_times = gt[:, 0]
    mask = (est[:, 0] >= gt_times[0]) & (est[:, 0] <= gt_times[-1])
    est_m = est[mask]
    if len(est_m) == 0:
        raise ValueError("no est samples within GT time range")
    # np.interp: 1D linear interpolation, per-axis.
    gt_pos = np.column_stack([
        np.interp(est_m[:, 0], gt_times, gt[:, 1]),
        np.interp(est_m[:, 0], gt_times, gt[:, 2]),
        np.interp(est_m[:, 0], gt_times, gt[:, 3]),
    ])
    return est_m, gt_pos


def umeyama(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """SE(3) alignment src -> dst (rotation + translation, no scale)."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    H = ((src - mu_s).T @ (dst - mu_d)) / src.shape[0]
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = mu_d - R @ mu_s
    return R, t


def segment_errors(
    est_matched: np.ndarray,
    gt_pos: np.ndarray,
    n_segments: int,
) -> list[SegmentResult]:
    est_pos = est_matched[:, 1:4]
    t = est_matched[:, 0]

    # Global alignment first — standard ATE practice; per-segment alignment
    # would mask exactly the late-heavy drift we want to detect.
    R, tr = umeyama(est_pos, gt_pos)
    est_aligned = (R @ est_pos.T).T + tr
    err = np.linalg.norm(est_aligned - gt_pos, axis=1)

    t0, t1 = t[0], t[-1]
    edges = np.linspace(t0, t1, n_segments + 1)
    results: list[SegmentResult] = []
    for i in range(n_segments):
        lo, hi = edges[i], edges[i + 1]
        if i < n_segments - 1:
            mask = (t >= lo) & (t < hi)
        else:
            mask = (t >= lo) & (t <= hi)
        seg_err = err[mask]
        if len(seg_err) == 0:
            continue
        results.append(
            SegmentResult(
                idx=i + 1,
                t_start=lo,
                t_end=hi,
                n_matches=int(len(seg_err)),
                rmse=float(np.sqrt(np.mean(seg_err ** 2))),
                mean=float(np.mean(seg_err)),
                max=float(np.max(seg_err)),
            )
        )
    return results


def classify(segs: Sequence[SegmentResult]) -> str:
    """Return one of 'uniform' | 'late-heavy' | 'early-heavy' | 'mid-heavy'."""
    if not segs:
        return "empty"
    rmses = [s.rmse for s in segs]
    if min(rmses) < 1e-9:
        return "degenerate"
    ratio = max(rmses) / min(rmses)
    if ratio < 2.0:
        return "uniform (contamination-dominant)"
    # find argmax segment
    peak_idx = int(np.argmax(rmses))
    n = len(segs)
    if peak_idx >= n - 1:
        return "late-heavy (loop-closure-absence-dominant)"
    if peak_idx == 0:
        return "early-heavy (init-drift-dominant)"
    return "mid-heavy (transient event)"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("est_csv")
    ap.add_argument("gt_txt")
    ap.add_argument("--segments", type=int, default=5)
    args = ap.parse_args()

    est = load_est(args.est_csv)
    gt = load_gt_tum(args.gt_txt)

    est_m, gt_pos = interpolate_gt(est, gt)
    segs = segment_errors(est_m, gt_pos, args.segments)

    print(f"est_csv:  {args.est_csv}")
    print(f"gt_txt:   {args.gt_txt}")
    print(f"n_est:    {len(est)}   n_gt: {len(gt)}   n_matches: {len(est_m)}")
    print(f"duration: {est_m[-1, 0] - est_m[0, 0]:.1f} s "
          f"over {args.segments} segments")
    print("")
    print(f"{'seg':>3}  {'t_start':>12}  {'t_end':>12}  {'n':>6}  "
          f"{'rmse':>7}  {'mean':>7}  {'max':>7}")
    for s in segs:
        print(f"{s.idx:>3}  {s.t_start:>12.2f}  {s.t_end:>12.2f}  "
              f"{s.n_matches:>6}  {s.rmse:>7.4f}  {s.mean:>7.4f}  {s.max:>7.4f}")

    rmses = [s.rmse for s in segs]
    print("")
    print(f"segment rmse summary: min={min(rmses):.4f}  "
          f"max={max(rmses):.4f}  ratio={max(rmses)/min(rmses):.2f}")
    print(f"classifier: {classify(segs)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
