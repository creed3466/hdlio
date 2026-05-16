#!/usr/bin/env python3
"""analyze_baseline.py — compile ATE + RPE + topview plots for 20260412 baseline.

For each of 27 sequences (Avia 9, Mid360 9, NTU VIRAL 9), load the dumped
trajectory + ground truth, compute:
  - ATE RMSE (Umeyama SE(3) alignment, same as eval_ate_m3dgr.py /
    eval_ate_ntu_viral.py)
  - RPE (relative pose error) at Δt=1.0 s over translation-only
  - GT-aligned topview PNG

Reuses the existing algorithms byte-for-byte; this script only orchestrates.
"""

from __future__ import annotations

import csv
import json
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

# Inside docker, the project is bind-mounted at /root/catkin_ws/; data under
# /root/catkin_ws/data/{m3dgr_surfel,ntu_viral} and dumps under /root/catkin_ws/dump.
REPO = Path(os.environ.get("REPO_ROOT", "/root/catkin_ws"))
DUMP_ROOT = REPO / "dump" / "bestset_20260411_1031"
VI04_POSTFIX = REPO / "dump" / "postfix_vi04_20260412_0324" / "Varying-illu04" / "run1"
GT_M3DGR = REPO / "data" / "m3dgr_surfel" / "ground_truth"
GT_NTU = REPO / "data" / "ntu_viral"
REPORT_DIR = REPO / "docs" / "reports" / "20260412_baseline"
PLOT_DIR = REPORT_DIR / "plots"


# NTU VIRAL lever arm (body → Leica prism), identical to eval_ate_ntu_viral.py
T_BODY_PRISM = np.array([-0.293656, -0.012288, -0.273095])


# ---------------------------------------------------------------------------
# Dataset configuration
# ---------------------------------------------------------------------------

M3DGR_SEQS = [
    "Dark01", "Dark02", "Dynamic03", "Dynamic04", "Occlusion03",
    "Occlusion04", "Varying-illu03", "Varying-illu04", "Varying-illu05",
]
NTU_SEQS = [
    "eee_01", "eee_02", "eee_03",
    "nya_01", "nya_02", "nya_03",
    "sbs_01", "sbs_02", "sbs_03",
]

# SOTA references (from run_bestset_all.sh). VI04 Avia comparator = VI05 paper
# number (0.199), not the decommissioned 0.102 phantom baseline.
SOTA_AVIA = {
    "Dark01": 0.118, "Dark02": 0.645, "Dynamic03": 0.151, "Dynamic04": 0.214,
    "Occlusion03": 0.315, "Occlusion04": 0.216, "Varying-illu03": 0.897,
    "Varying-illu04": 0.199, "Varying-illu05": 0.245,
}
SOTA_MID = {
    "Dark01": 0.173, "Dark02": 0.212, "Dynamic03": 0.178, "Dynamic04": 0.214,
    "Occlusion03": 0.315, "Occlusion04": 0.216, "Varying-illu03": 0.957,
    "Varying-illu04": 0.161, "Varying-illu05": 0.245,
}
SOTA_NTU = {
    "eee_01": 0.131, "eee_02": 0.124, "eee_03": 0.163,
    "nya_01": 0.122, "nya_02": 0.142, "nya_03": 0.144,
    "sbs_01": 0.142, "sbs_02": 0.140, "sbs_03": 0.133,
}


@dataclass
class SeqResult:
    dataset: str
    seq: str
    ate_rmse: float
    ate_mean: float
    ate_median: float
    ate_std: float
    ate_max: float
    rpe_trans_rmse: float
    rpe_trans_mean: float
    rpe_trans_median: float
    rpe_trans_max: float
    rpe_delta_s: float
    n_matches: int
    n_rpe_pairs: int
    traj_len_m: float
    duration_s: float
    sota: Optional[float] = None
    source: str = "bestset"


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------


def load_est_traj(path: Path) -> np.ndarray:
    rows = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        next(reader, None)  # header
        for row in reader:
            if len(row) < 8:
                continue
            rows.append([float(v) for v in row[:8]])
    return np.array(rows)


def load_gt_m3dgr(path: Path) -> np.ndarray:
    """Space-separated TUM: t x y z qx qy qz qw."""
    rows = []
    with open(path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            try:
                rows.append([float(v) for v in parts[:8]])
            except ValueError:
                continue
    return np.array(rows)


def load_gt_ntu(path: Path) -> np.ndarray:
    """NTU VIRAL ground_truth.csv: %time,seq,stamp,x,y,z,qx,qy,qz,qw (ns stamp)."""
    rows = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) < 10:
                continue
            t = float(row[2]) * 1e-9
            rows.append([t, float(row[3]), float(row[4]), float(row[5]),
                         float(row[6]), float(row[7]), float(row[8]), float(row[9])])
    return np.array(rows)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def apply_lever_arm(est: np.ndarray, t_body_prism: np.ndarray) -> np.ndarray:
    """Transform body-frame SLAM position → prism frame using estimated R."""
    out = est.copy()
    quats = est[:, 4:8]  # qx qy qz qw
    R_wb = Rotation.from_quat(quats)
    out[:, 1:4] += R_wb.apply(t_body_prism)
    return out


def umeyama(est_pos: np.ndarray, gt_pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    assert est_pos.shape == gt_pos.shape
    n = est_pos.shape[0]
    mu_e = est_pos.mean(axis=0)
    mu_g = gt_pos.mean(axis=0)
    H = (est_pos - mu_e).T @ (gt_pos - mu_g) / n
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = mu_g - R @ mu_e
    return R, t


def interp_gt(est: np.ndarray, gt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    est_t = est[:, 0]
    gt_t = gt[:, 0]
    mask = (est_t >= gt_t[0]) & (est_t <= gt_t[-1])
    est_m = est[mask]
    if len(est_m) == 0:
        return np.empty((0, 8)), np.empty((0, 3))
    interp = interp1d(gt_t, gt[:, 1:4], axis=0, kind="linear")
    gt_pos_i = interp(est_m[:, 0])
    return est_m, gt_pos_i


def compute_ate(est_pos: np.ndarray, gt_pos: np.ndarray) -> dict:
    R, t = umeyama(est_pos, gt_pos)
    aligned = (R @ est_pos.T).T + t
    errs = np.linalg.norm(aligned - gt_pos, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(errs ** 2))),
        "mean": float(errs.mean()),
        "median": float(np.median(errs)),
        "std": float(errs.std()),
        "max": float(errs.max()),
        "aligned": aligned,
        "R": R,
        "t": t,
    }


# ---------------------------------------------------------------------------
# RPE (translation RMSE at fixed Δt, rotation RMSE in deg)
# ---------------------------------------------------------------------------


def compute_rpe(est_aligned_pos: np.ndarray, gt_pos: np.ndarray,
                times: np.ndarray, delta_s: float = 1.0) -> dict:
    """
    Translation RPE at fixed temporal Δt.

    For each frame i, find first j with t_j ≥ t_i + Δt (and actual dt < 1.5·Δt).
    Both est and GT are in the same world frame — the estimate is the
    **Umeyama-aligned** trajectory, so global rotation/translation has already
    been removed. The RPE captures local drift only:

        err_i = ‖ (p_est_j − p_est_i) − (p_gt_j − p_gt_i) ‖

    Rotation RPE is not reported because both M3DGR (VRPN/Optitrack) and
    NTU VIRAL (Leica prism) ground truths carry identity quaternions —
    orientation is not observed by either ground-truth source.
    """
    if len(times) < 3:
        return dict(trans_rmse=float("nan"), trans_mean=float("nan"),
                    trans_median=float("nan"), trans_max=float("nan"),
                    n_pairs=0, delta_s=delta_s)

    n = len(times)
    j_idx = np.searchsorted(times, times + delta_s, side="left")
    valid = j_idx < n
    i_list = np.arange(n)[valid]
    j_list = j_idx[valid]
    if len(i_list) == 0:
        return dict(trans_rmse=float("nan"), trans_mean=float("nan"),
                    trans_median=float("nan"), trans_max=float("nan"),
                    n_pairs=0, delta_s=delta_s)

    dt_actual = times[j_list] - times[i_list]
    good = dt_actual < 1.5 * delta_s
    i_list = i_list[good]
    j_list = j_list[good]
    if len(i_list) < 10:
        return dict(trans_rmse=float("nan"), trans_mean=float("nan"),
                    trans_median=float("nan"), trans_max=float("nan"),
                    n_pairs=int(len(i_list)), delta_s=delta_s)

    d_est = est_aligned_pos[j_list] - est_aligned_pos[i_list]
    d_gt = gt_pos[j_list] - gt_pos[i_list]
    err = np.linalg.norm(d_est - d_gt, axis=1)

    return dict(trans_rmse=float(np.sqrt(np.mean(err ** 2))),
                trans_mean=float(err.mean()),
                trans_median=float(np.median(err)),
                trans_max=float(err.max()),
                n_pairs=int(len(i_list)),
                delta_s=delta_s)


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------


def plot_topview(est_aligned: np.ndarray, gt_pos: np.ndarray,
                 title: str, ate_rmse: float, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(gt_pos[:, 0], gt_pos[:, 1], color="#888888", lw=1.6,
            label="GT", zorder=1)
    ax.plot(est_aligned[:, 0], est_aligned[:, 1], color="#1f77b4", lw=1.2,
            label=f"est (ATE={ate_rmse:.3f} m)", zorder=2)
    # mark start / end
    ax.scatter(gt_pos[0, 0], gt_pos[0, 1], c="#2ca02c", s=40, zorder=3,
               label="start")
    ax.scatter(gt_pos[-1, 0], gt_pos[-1, 1], c="#d62728", s=40, zorder=3,
               label="end")
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Per-sequence processing
# ---------------------------------------------------------------------------


def process_seq(dataset: str, seq: str) -> Optional[SeqResult]:
    # Resolve trajectory source
    if dataset == "avia" and seq == "Varying-illu04":
        traj_path = VI04_POSTFIX / "traj.csv"
        source = "postfix_vi04_20260412_0324/run1"
    elif dataset == "avia":
        traj_path = DUMP_ROOT / "avia" / seq / "traj.csv"
        source = "bestset_20260411_1031/avia"
    elif dataset == "mid360":
        traj_path = DUMP_ROOT / "mid360" / seq / "traj.csv"
        source = "bestset_20260411_1031/mid360"
    elif dataset == "ntu":
        traj_path = DUMP_ROOT / "ntu_viral" / seq / "traj.csv"
        source = "bestset_20260411_1031/ntu_viral"
    else:
        return None

    if not traj_path.exists():
        print(f"  [skip] {dataset}/{seq} — no traj.csv", file=sys.stderr)
        return None

    est = load_est_traj(traj_path)
    if est.size == 0:
        print(f"  [skip] {dataset}/{seq} — empty traj", file=sys.stderr)
        return None

    # GT + lever arm
    if dataset in ("avia", "mid360"):
        gt = load_gt_m3dgr(GT_M3DGR / f"{seq}.txt")
        est_eval = est
    elif dataset == "ntu":
        gt = load_gt_ntu(GT_NTU / seq / "ground_truth.csv")
        est_eval = apply_lever_arm(est, T_BODY_PRISM)
    else:
        return None

    est_m, gt_pos_i = interp_gt(est_eval, gt)
    if len(est_m) < 10:
        print(f"  [skip] {dataset}/{seq} — only {len(est_m)} matches", file=sys.stderr)
        return None

    ate = compute_ate(est_m[:, 1:4], gt_pos_i)
    # RPE uses the Umeyama-aligned est positions so that global rotation has
    # been removed before computing local increments.
    rpe = compute_rpe(ate["aligned"], gt_pos_i, est_m[:, 0], delta_s=1.0)

    # Trajectory length (estimate, unaligned)
    diffs = np.diff(est_m[:, 1:4], axis=0)
    traj_len = float(np.linalg.norm(diffs, axis=1).sum())
    duration = float(est_m[-1, 0] - est_m[0, 0])

    # Plot
    ds_dir = {"avia": "avia", "mid360": "mid360", "ntu": "ntu_viral"}[dataset]
    plot_out = PLOT_DIR / ds_dir / f"{seq}.png"
    plot_out.parent.mkdir(parents=True, exist_ok=True)
    plot_topview(ate["aligned"], gt_pos_i,
                 f"{dataset} / {seq}", ate["rmse"], plot_out)

    sota_map = {"avia": SOTA_AVIA, "mid360": SOTA_MID, "ntu": SOTA_NTU}[dataset]

    return SeqResult(
        dataset=dataset, seq=seq,
        ate_rmse=ate["rmse"], ate_mean=ate["mean"], ate_median=ate["median"],
        ate_std=ate["std"], ate_max=ate["max"],
        rpe_trans_rmse=rpe["trans_rmse"], rpe_trans_mean=rpe["trans_mean"],
        rpe_trans_median=rpe["trans_median"], rpe_trans_max=rpe["trans_max"],
        rpe_delta_s=rpe["delta_s"],
        n_matches=int(len(est_m)), n_rpe_pairs=int(rpe["n_pairs"]),
        traj_len_m=traj_len, duration_s=duration,
        sota=sota_map.get(seq), source=source,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    datasets = [
        ("avia", M3DGR_SEQS),
        ("mid360", M3DGR_SEQS),
        ("ntu", NTU_SEQS),
    ]

    all_results: list[SeqResult] = []
    for ds, seqs in datasets:
        print(f"[{ds}]")
        for seq in seqs:
            r = process_seq(ds, seq)
            if r is None:
                continue
            beat_marker = ""
            if r.sota is not None:
                beat_marker = "✓" if r.ate_rmse <= r.sota else "✗"
            print(f"  {seq:<16s} ATE={r.ate_rmse:.4f}  RPE_t={r.rpe_trans_rmse:.4f}"
                  f"  SOTA={r.sota}  {beat_marker}")
            all_results.append(r)

    # Serialise to JSON for the markdown writer
    out_json = REPORT_DIR / "results.json"
    with open(out_json, "w") as f:
        json.dump([r.__dict__ for r in all_results], f, indent=2,
                  default=lambda o: None)
    print(f"\nWrote {out_json}  ({len(all_results)} rows)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
