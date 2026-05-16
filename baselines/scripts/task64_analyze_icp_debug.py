#!/usr/bin/env python3
"""task64_analyze_icp_debug.py

Analyse per-frame ICP diagnostic CSV produced by the TOFSLAM_ICP_DEBUG=1
instrumentation in baselines/algorithms/ig_lio/src/lio.cpp, against the
iG-LIO trajectory and NTU VIRAL ground truth, to decide H_B (ICP
correspondence degradation in sparse/foliage zones on sbs_03).

Three temporal windows (seconds relative to bag start, i.e.
timestamp - first_frame_timestamp):
  W1  0 –   15 s   (stable prefix)
  W2 15 –   45 s   (error-spike, suspected foliage — H_B target window)
  W3 45 s  – end   (recovery / convergence)

For each window we report mean, std of:
  - n_corr_inlier
  - mean_residual
  - map_voxel_count

Verdict rule (H_B acceptance):
  W2 inlier count is at least 2 sigma below the pooled W1/W3 baseline
  AND (W2 mean_residual ≥ 2 sigma above the pooled W1/W3 baseline
       OR W2 map_voxel_count ≥ 2 sigma below pooled W1/W3 baseline).

Outputs:
  - baselines/docs/task64_icp_analysis_20260414.md   (table + verdict)
  - paper_figures/task64_sbs03_icp_timeline.png      (scatter overlay)
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "paper_figures"))
from alignment import (  # type: ignore  # noqa: E402
    GT_GAP_THRESHOLD_S,
    T_BODY_PRISM,  # noqa: F401
    apply_lever_arm,
    compute_ate,
    interp_gt,
)

W1_END_S = 15.0
W2_END_S = 45.0


@dataclass(frozen=True)
class WindowStats:
    label: str
    n: int
    mean_inlier: float
    std_inlier: float
    mean_residual: float
    std_residual: float
    mean_voxels: float
    std_voxels: float


def load_icp_csv(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    if data.size == 0:
        raise RuntimeError(f"empty CSV: {path}")
    return data


def load_tum(path: Path) -> np.ndarray:
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
    return np.asarray(rows)


def window_stats(
    label: str,
    rel_t: np.ndarray,
    inlier: np.ndarray,
    residual: np.ndarray,
    voxels: np.ndarray,
    t_lo: float,
    t_hi: float,
) -> WindowStats:
    mask = (rel_t >= t_lo) & (rel_t < t_hi)
    if not mask.any():
        return WindowStats(label, 0, *([float("nan")] * 6))
    return WindowStats(
        label=label,
        n=int(mask.sum()),
        mean_inlier=float(inlier[mask].mean()),
        std_inlier=float(inlier[mask].std(ddof=1)) if mask.sum() > 1 else 0.0,
        mean_residual=float(residual[mask].mean()),
        std_residual=float(residual[mask].std(ddof=1)) if mask.sum() > 1 else 0.0,
        mean_voxels=float(voxels[mask].mean()),
        std_voxels=float(voxels[mask].std(ddof=1)) if mask.sum() > 1 else 0.0,
    )


def pooled_mean_std(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    pooled = np.concatenate([a, b])
    if pooled.size == 0:
        return float("nan"), float("nan")
    return float(pooled.mean()), float(pooled.std(ddof=1)) if pooled.size > 1 else 0.0


def verdict(
    w1: WindowStats,
    w2: WindowStats,
    w3: WindowStats,
    rel_t: np.ndarray,
    inlier: np.ndarray,
    residual: np.ndarray,
    voxels: np.ndarray,
) -> tuple[str, dict]:
    mask13 = (rel_t < W1_END_S) | (rel_t >= W2_END_S)
    maskW2 = (rel_t >= W1_END_S) & (rel_t < W2_END_S)
    if not mask13.any() or not maskW2.any():
        return "inconclusive", {"reason": "window coverage insufficient"}
    base_inlier = inlier[mask13]
    base_residual = residual[mask13]
    base_voxels = voxels[mask13]

    def z(x: float, base: np.ndarray) -> float:
        mu = float(base.mean())
        sigma = float(base.std(ddof=1)) if base.size > 1 else 0.0
        if sigma == 0.0:
            return float("nan")
        return (x - mu) / sigma

    z_inlier = z(float(inlier[maskW2].mean()), base_inlier)
    z_residual = z(float(residual[maskW2].mean()), base_residual)
    z_voxels = z(float(voxels[maskW2].mean()), base_voxels)

    accept_inlier = not np.isnan(z_inlier) and z_inlier <= -2.0
    accept_residual = not np.isnan(z_residual) and z_residual >= 2.0
    accept_voxels = not np.isnan(z_voxels) and z_voxels <= -2.0

    diag = {
        "z_W2_inlier_vs_W1W3": z_inlier,
        "z_W2_residual_vs_W1W3": z_residual,
        "z_W2_voxels_vs_W1W3": z_voxels,
        "criterion_inlier_<=-2": accept_inlier,
        "criterion_residual_>=+2": accept_residual,
        "criterion_voxels_<=-2": accept_voxels,
    }

    if accept_inlier and (accept_residual or accept_voxels):
        return "accept", diag
    # Rebut if none of the three z-thresholds clear 2 sigma AND signs are
    # all benign (inlier z > -0.5, residual z < 0.5, voxels z > -0.5).
    if (
        (np.isnan(z_inlier) or z_inlier > -0.5)
        and (np.isnan(z_residual) or z_residual < 0.5)
        and (np.isnan(z_voxels) or z_voxels > -0.5)
    ):
        return "rebut", diag
    return "inconclusive", diag


def per_pose_error(traj_path: Path, gt_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (est_timestamps, per_pose_error_m) using the NTU VIRAL
    lever-arm + Umeyama alignment used in compute_ate.py."""
    est = load_tum(traj_path)
    gt = load_tum(gt_path)
    est = apply_lever_arm(est)
    est_m, gt_pos = interp_gt(est, gt, gap_threshold_s=GT_GAP_THRESHOLD_S)
    if len(est_m) < 10:
        return np.empty(0), np.empty(0)
    result = compute_ate(est_m[:, 1:4], gt_pos)
    errors = np.asarray(result["errors"])
    return est_m[:, 0], errors


def make_plot(
    csv_rel_t: np.ndarray,
    csv_inlier: np.ndarray,
    csv_residual: np.ndarray,
    err_rel_t: np.ndarray,
    errors: np.ndarray,
    out_png: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax1 = plt.subplots(figsize=(11, 5.5))
    ax1.scatter(csv_rel_t, csv_inlier, s=5, alpha=0.5, color="tab:blue", label="ICP inliers")
    ax1.set_xlabel("time since bag start (s)")
    ax1.set_ylabel("ICP inlier correspondences", color="tab:blue")
    ax1.tick_params(axis="y", labelcolor="tab:blue")

    ax2 = ax1.twinx()
    if errors.size > 0:
        ax2.plot(err_rel_t, errors, color="tab:red", alpha=0.85, linewidth=1.2, label="ATE error (m)")
    ax2.set_ylabel("ATE per-pose error (m)", color="tab:red")
    ax2.tick_params(axis="y", labelcolor="tab:red")

    for t_boundary, lbl in [(W1_END_S, "W1|W2"), (W2_END_S, "W2|W3")]:
        ax1.axvline(t_boundary, color="k", linestyle="--", alpha=0.35)
        ax1.text(t_boundary + 0.3, ax1.get_ylim()[1] * 0.95, lbl, fontsize=9, alpha=0.7)

    ax1.set_title("iG-LIO sbs_03 — ICP inlier count vs ATE error (task #64, H_B diagnostic)")
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def format_markdown(
    w1: WindowStats,
    w2: WindowStats,
    w3: WindowStats,
    verdict_label: str,
    diag: dict,
    csv_path: Path,
    n_rows: int,
    run_duration_s: float,
) -> str:
    lines: list[str] = []
    lines.append("# Task #64 — ICP Debug Analysis on sbs_03 (H_B)")
    lines.append("")
    lines.append("**Date**: 2026-04-14  ")
    lines.append("**Hypothesis under test**: H_B — ICP correspondence degrades in the")
    lines.append("sparse/foliage courtyard segment (t=15–45 s) on NTU sbs_03.  ")
    lines.append(f"**Data source**: `{csv_path}` ({n_rows} lidar frames, duration={run_duration_s:.1f} s)  ")
    lines.append("**Instrumentation**: ig_lio commits `task #64 (a)` + `task #64 (b)`, env-gated via `TOFSLAM_ICP_DEBUG=1`.")
    lines.append("")
    lines.append("## 3-Window Statistics")
    lines.append("")
    lines.append("| Window | t-range | Frames | Inliers (mean ± std) | Mean residual (mean ± std) | Map voxels (mean ± std) |")
    lines.append("|--------|---------|-------:|---------------------:|---------------------------:|------------------------:|")
    for w in (w1, w2, w3):
        lines.append(
            f"| **{w.label}** | see header | {w.n} | "
            f"{w.mean_inlier:.1f} ± {w.std_inlier:.1f} | "
            f"{w.mean_residual:.4g} ± {w.std_residual:.4g} | "
            f"{w.mean_voxels:.0f} ± {w.std_voxels:.0f} |"
        )
    lines.append("")
    lines.append("Window definitions:  ")
    lines.append(f"- **W1**: 0 ≤ t < {W1_END_S:.0f} s (stable prefix)  ")
    lines.append(f"- **W2**: {W1_END_S:.0f} ≤ t < {W2_END_S:.0f} s (suspected foliage / error spike)  ")
    lines.append(f"- **W3**: t ≥ {W2_END_S:.0f} s (recovery)")
    lines.append("")
    lines.append("## Z-score of W2 vs pooled W1∪W3 baseline")
    lines.append("")
    lines.append("| Metric | W2 z-score | Accept threshold | Criterion met |")
    lines.append("|--------|-----------:|:-----------------|:-------------:|")
    lines.append(
        f"| Inlier count    | {diag.get('z_W2_inlier_vs_W1W3', float('nan')):+.2f} | ≤ −2.0 "
        f"| {'YES' if diag.get('criterion_inlier_<=-2') else 'no'} |"
    )
    lines.append(
        f"| Mean residual   | {diag.get('z_W2_residual_vs_W1W3', float('nan')):+.2f} | ≥ +2.0 "
        f"| {'YES' if diag.get('criterion_residual_>=+2') else 'no'} |"
    )
    lines.append(
        f"| Map voxel count | {diag.get('z_W2_voxels_vs_W1W3', float('nan')):+.2f} | ≤ −2.0 "
        f"| {'YES' if diag.get('criterion_voxels_<=-2') else 'no'} |"
    )
    lines.append("")
    lines.append(f"## Verdict: **H_B {verdict_label.upper()}**")
    lines.append("")
    if verdict_label == "accept":
        lines.append("W2 inlier count is ≥ 2σ below the W1∪W3 baseline **and** at least one")
        lines.append("of residual/voxel-count metrics confirms the degradation. This is the")
        lines.append("expected signature of ICP correspondence collapse in sparse/foliage")
        lines.append("zones. Proceed to Architect stage to design a remediation (multi-")
        lines.append("resolution ICP, adaptive gain, keyframe density floor, etc.).")
    elif verdict_label == "rebut":
        lines.append("No systematic degradation of ICP correspondences is visible in the")
        lines.append("t=15–45 s window relative to the surrounding windows. H_B is")
        lines.append("rejected. Return to Research to examine H_C (voxel resolution tuning)")
        lines.append("or H_D (EKF gate rejection).")
    else:
        lines.append("The signature is mixed — one or two criteria trend in the expected")
        lines.append("direction but do not clear 2σ, or a metric moves opposite to the")
        lines.append("prediction. Extend the window definition (e.g. align to the error")
        lines.append("peak rather than the prior task53 estimate) or add H_D (EKF gate)")
        lines.append("instrumentation before drawing a conclusion.")
    lines.append("")
    lines.append("## Artifacts")
    lines.append("")
    lines.append(f"- ICP debug CSV: `{csv_path}`")
    lines.append(f"- Plot: `paper_figures/task64_sbs03_icp_timeline.png`")
    lines.append("- Research: `docs/reports/task64_sbs03_non_init_research_20260414.md`")
    lines.append("")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--traj", required=True, type=Path)
    parser.add_argument("--gt", required=True, type=Path)
    parser.add_argument("--out-md", required=True, type=Path)
    parser.add_argument("--out-png", required=True, type=Path)
    args = parser.parse_args()

    rows = load_icp_csv(args.csv)
    ts = np.asarray(rows["timestamp"], dtype=float)
    inlier = np.asarray(rows["n_corr_inlier"], dtype=float)
    residual = np.asarray(rows["mean_residual"], dtype=float)
    voxels = np.asarray(rows["map_voxel_count"], dtype=float)

    if ts.size == 0:
        print("[ERROR] no ICP rows — instrumentation may not have fired")
        return 2
    t0 = float(ts.min())
    rel_t = ts - t0

    w1 = window_stats("W1 (0-15s)", rel_t, inlier, residual, voxels, 0.0, W1_END_S)
    w2 = window_stats("W2 (15-45s)", rel_t, inlier, residual, voxels, W1_END_S, W2_END_S)
    w3 = window_stats("W3 (>=45s)", rel_t, inlier, residual, voxels, W2_END_S, float("inf"))

    verdict_label, diag = verdict(w1, w2, w3, rel_t, inlier, residual, voxels)

    err_t, err_vals = per_pose_error(args.traj, args.gt)
    err_rel = err_t - t0 if err_t.size > 0 else err_t

    make_plot(rel_t, inlier, residual, err_rel, err_vals, args.out_png)

    md = format_markdown(
        w1, w2, w3,
        verdict_label, diag,
        args.csv,
        n_rows=int(ts.size),
        run_duration_s=float(rel_t.max()) if rel_t.size else 0.0,
    )
    args.out_md.parent.mkdir(parents=True, exist_ok=True)
    args.out_md.write_text(md)

    print(f"[ok] wrote {args.out_md}")
    print(f"[ok] wrote {args.out_png}")
    print(f"[verdict] H_B: {verdict_label.upper()}")
    for k, v in diag.items():
        print(f"  {k}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
