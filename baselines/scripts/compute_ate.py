#!/usr/bin/env python3
"""compute_ate.py — ATE RMSE between TUM-format trajectory and GT.

Usage:
    compute_ate.py <traj_tum> <gt_tum> [--out ate.json]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# Re-use alignment utilities from paper_figures
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "paper_figures"))
from alignment import (  # type: ignore  # noqa: E402
    GT_GAP_THRESHOLD_S,
    T_BODY_PRISM,
    apply_lever_arm,
    compute_ate,
    interp_gt,
)


def load_tum(path: Path) -> np.ndarray:
    """Load whitespace-separated TUM file: t x y z qx qy qz qw."""
    rows: list[list[float]] = []
    with open(path, "r") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                rows.append([float(p) for p in parts[:8]])
            except ValueError:
                continue
    if not rows:
        raise RuntimeError(f"No TUM rows parsed from {path}")
    return np.asarray(rows)


def _is_ntu_viral_gt(gt_path: Path) -> bool:
    """Detect NTU VIRAL GT by path pattern (dump/ntu_gt_tum/*_gt.tum)."""
    parts = {p.lower() for p in gt_path.parts}
    return "ntu_gt_tum" in parts or gt_path.stem.endswith("_gt") and "ntu" in str(gt_path).lower()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("traj", type=Path)
    parser.add_argument("gt", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument(
        "--ntu-viral",
        dest="ntu_viral",
        action="store_true",
        default=None,
        help="Apply NTU VIRAL prism lever-arm correction (T_body_prism) "
        "before ATE. Autodetected from gt path when flag is omitted.",
    )
    parser.add_argument(
        "--no-ntu-viral",
        dest="ntu_viral",
        action="store_false",
        help="Force-disable lever-arm autodetect for NTU VIRAL paths.",
    )
    parser.add_argument(
        "--gt-gap-threshold-s",
        type=float,
        default=GT_GAP_THRESHOLD_S,
        help=(
            "Drop est samples whose nearest GT neighbour is further than "
            "this many seconds away (default: %(default)s). NTU VIRAL's "
            "Leica GT has multi-second dropouts on SBS sequences; linear "
            "interp over these gaps produces phantom ATE spikes. Pass 0 "
            "to disable the filter (legacy behaviour)."
        ),
    )
    args = parser.parse_args()

    est = load_tum(args.traj)
    gt = load_tum(args.gt)
    print(f"[info] est poses: {len(est)}   gt poses: {len(gt)}")

    # NTU VIRAL: GT is in Leica prism frame; SLAM output is body/IMU frame.
    # Per Nguyen et al. IJRR 2022 evaluation tutorial + SLICT RA-L 2023,
    # the authoritative protocol applies T_body_prism per-pose (rotated by
    # est orientation) BEFORE Umeyama alignment. Reference impls:
    # docker/analyze_baseline.py:333, docker/eval_ate_ntu_viral.py:263.
    ntu_viral = args.ntu_viral
    if ntu_viral is None:
        ntu_viral = _is_ntu_viral_gt(args.gt)
    if ntu_viral:
        print(
            f"[info] NTU VIRAL lever-arm: applying T_body_prism="
            f"{T_BODY_PRISM.tolist()} (||=0.40 m) per-pose before ATE"
        )
        est = apply_lever_arm(est)

    n_est_before = int(((est[:, 0] >= gt[0, 0]) & (est[:, 0] <= gt[-1, 0])).sum())
    est_m, gt_pos = interp_gt(est, gt, gap_threshold_s=args.gt_gap_threshold_s)
    n_dropped_gap = max(0, n_est_before - len(est_m))
    if args.gt_gap_threshold_s and n_dropped_gap:
        print(
            f"[info] GT-gap filter (>{args.gt_gap_threshold_s}s): "
            f"dropped {n_dropped_gap}/{n_est_before} samples "
            f"({100 * n_dropped_gap / max(1, n_est_before):.1f}%)"
        )
    if len(est_m) < 10:
        print(f"[ERROR] Too few overlapping timestamps: {len(est_m)}")
        return 2

    result = compute_ate(est_m[:, 1:4], gt_pos)
    summary = {
        "rmse_m": result["rmse"],
        "mean_m": result["mean"],
        "median_m": result["median"],
        "std_m": result["std"],
        "max_m": result["max"],
        "n_poses": len(est_m),
        "est_file": str(args.traj),
        "gt_file": str(args.gt),
        "ntu_viral_lever_arm": bool(ntu_viral),
        "gt_gap_threshold_s": float(args.gt_gap_threshold_s),
        "n_dropped_gt_gap": int(n_dropped_gap),
    }
    print(json.dumps(summary, indent=2))

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(summary, indent=2))
        print(f"[ok] wrote {args.out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
