#!/usr/bin/env python3
"""Task #71 — aggregate per-seq ATE into a single iteration summary CSV.

Reads `ate_result.txt` files from `dump/iter<N>/<dataset>/<seq>/` and emits
`dump/iter<N>/summary.csv` with the schema documented in
`docs/adbe_protocol.md`.

Usage:
    python3 docker/collect_iter_summary.py <ITER_N> [--prev-iter <M>]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Iterable

# Baseline numbers from docs/reports/full_eval_20260414.md (08933ab).
BASELINE: dict[str, float] = {
    # Avia
    "Dark01": 0.14629949601951586,
    "Dark02": 0.65550107918863490,
    "Dynamic03": 0.17652382898916288,
    "Dynamic04": 0.31868196686208483,
    "Occlusion03": 0.34722369490160615,
    "Occlusion04": 0.30981956405698285,
    "Varying-illu03": 1.05554282542740420,
    "Varying-illu04": 0.25317561957825946,
    "Varying-illu05": 0.18461554312386352,
}

MID360_BASELINE: dict[str, float] = {
    "Varying-illu04": 0.30980626606390130,
    "Varying-illu03": 1.23130791250331570,
    "Dark02": 0.27905714108469040,
}

SOTA: dict[str, dict[str, float]] = {
    "avia": {
        "Dark01": 0.118, "Dark02": 0.645, "Dynamic03": 0.151, "Dynamic04": 0.214,
        "Occlusion03": 0.315, "Occlusion04": 0.216,
        "Varying-illu03": 0.897, "Varying-illu04": 0.199, "Varying-illu05": 0.245,
    },
    "mid360": {
        "Varying-illu04": 0.161, "Varying-illu03": 0.957, "Dark02": 0.212,
    },
}

AVIA_SEQS = list(BASELINE.keys())
MID360_LAG_SEQS = list(MID360_BASELINE.keys())


def read_rmse(ate_file: Path) -> float | None:
    if not ate_file.exists():
        return None
    for line in ate_file.read_text().splitlines():
        m = re.match(r"^\s*rmse:\s*(\S+)", line)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                return None
    return None


def load_prev_rmse(prev_summary: Path) -> dict[tuple[str, str], float]:
    result: dict[tuple[str, str], float] = {}
    if not prev_summary.exists():
        return result
    with prev_summary.open() as fp:
        reader = csv.DictReader(fp)
        for row in reader:
            try:
                result[(row["dataset"], row["seq"])] = float(row["rmse_m"])
            except (KeyError, ValueError):
                pass
    return result


def gather(iter_n: int, prev_iter: int | None) -> None:
    base = Path(f"dump/iter{iter_n}")
    out = base / "summary.csv"
    manifest = base / "manifest.json"
    prev_map: dict[tuple[str, str], float] = {}
    if prev_iter is not None:
        prev_map = load_prev_rmse(Path(f"dump/iter{prev_iter}/summary.csv"))

    git_sha = subprocess.run(
        ["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=False
    ).stdout.strip()
    run_utc = subprocess.run(
        ["date", "-u", "+%Y-%m-%dT%H:%M:%SZ"], capture_output=True, text=True, check=False
    ).stdout.strip()

    rows: list[dict[str, str]] = []
    for dataset, baseline, seqs in (
        ("avia", BASELINE, AVIA_SEQS),
        ("mid360", MID360_BASELINE, MID360_LAG_SEQS),
    ):
        for seq in seqs:
            ate_file = base / dataset / seq / "ate_result.txt"
            rmse = read_rmse(ate_file)
            base_rmse = baseline[seq]
            sota_rmse = SOTA[dataset][seq]
            prev_rmse = prev_map.get((dataset, seq))
            rows.append({
                "iter": str(iter_n),
                "seq": seq,
                "dataset": dataset,
                "rmse_m": f"{rmse:.17g}" if rmse is not None else "",
                "baseline_rmse": f"{base_rmse:.17g}",
                "prev_iter_rmse": f"{prev_rmse:.17g}" if prev_rmse is not None else "",
                "delta_vs_baseline": f"{(rmse - base_rmse):.6g}" if rmse is not None else "",
                "delta_vs_prev": (
                    f"{(rmse - prev_rmse):.6g}" if rmse is not None and prev_rmse is not None else ""
                ),
                "sota_rmse": f"{sota_rmse:.6g}",
                "sota_ratio": f"{(rmse / sota_rmse):.6g}" if rmse is not None else "",
                "rate": "1.0",
                "commit_sha": git_sha,
                "run_utc": run_utc,
            })

    fields = [
        "iter", "seq", "dataset", "rmse_m", "baseline_rmse", "prev_iter_rmse",
        "delta_vs_baseline", "delta_vs_prev", "sota_rmse", "sota_ratio",
        "rate", "commit_sha", "run_utc",
    ]
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)

    # Summary metrics
    avia_vals = [float(r["rmse_m"]) for r in rows if r["dataset"] == "avia" and r["rmse_m"]]
    mid_vals = [float(r["rmse_m"]) for r in rows if r["dataset"] == "mid360" and r["rmse_m"]]
    avia_mean = sum(avia_vals) / len(avia_vals) if avia_vals else float("nan")
    mid_mean = sum(mid_vals) / len(mid_vals) if mid_vals else float("nan")
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps({
        "iter": iter_n,
        "commit_sha": git_sha,
        "run_utc": run_utc,
        "avia_seqs_present": len(avia_vals),
        "avia_mean_rmse": avia_mean,
        "mid360_lag_seqs_present": len(mid_vals),
        "mid360_lag_mean_rmse": mid_mean,
        "target_avia_mean": 0.300,
    }, indent=2))

    # Print decision-ready line to stdout
    sota_avia_mean = sum(SOTA["avia"][s] for s in AVIA_SEQS) / len(AVIA_SEQS)
    print(f"Avia Mean RMSE: {avia_mean:.4f} m   (SOTA Mean {sota_avia_mean:.4f}, "
          f"target ≤ {0.9 * sota_avia_mean:.4f})")
    print(f"Mid360-lag Mean RMSE: {mid_mean:.4f} m")
    print(f"CSV: {out}")
    print(f"Manifest: {manifest}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("iter", type=int)
    ap.add_argument("--prev-iter", type=int, default=None)
    args = ap.parse_args()
    gather(args.iter, args.prev_iter)


if __name__ == "__main__":
    main()
