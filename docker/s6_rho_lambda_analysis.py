#!/usr/bin/env python3
"""S6-R1.2 ρ_λ measurement campaign analysis.

Processes diagnostics.csv per sequence, pools eigenvalue_ratio by
(sensor, domain), computes percentile / Otsu discriminators, and
emits ρ̂_ref recommendations for the severity-scaled Eq.(4).

Outputs:
  rho_lambda_summary.json   — machine-readable percentiles + discriminators
  rho_lambda_report.md      — human-readable report
  rho_lambda_avia.png       — histogram (if matplotlib available)
  rho_lambda_mid360.png     — histogram (if matplotlib available)

Usage:
  python3 docker/s6_rho_lambda_analysis.py dump/S6_rho_campaign
"""

from __future__ import annotations
import csv
import json
import math
import sys
from pathlib import Path

try:
    import numpy as np  # type: ignore
    HAVE_NP = True
except Exception:
    HAVE_NP = False

# ---------------------------------------------------------------------------
# Sequence classification (sensor × domain)
# ---------------------------------------------------------------------------

AVIA_OUTDOOR = {
    "Dark01", "Dark02", "Dynamic03", "Dynamic04",
    "Occlusion03", "Occlusion04",
    "Varying-illu03", "Varying-illu04", "Varying-illu05",
}
AVIA_INDOOR = {
    "indoor_Dark03", "indoor_Dark04",
    "indoor_Dynamic01", "indoor_Dynamic02",
    "indoor_Occlusion01", "indoor_Occlusion02",
    "indoor_Varying-illu01", "indoor_Varying-illu02",
}
MID360_OUTDOOR = {f"m_{s}" for s in AVIA_OUTDOOR}
MID360_INDOOR  = {f"m_{s}" for s in AVIA_INDOOR}

CLASSIFICATION = {
    "avia_outdoor": AVIA_OUTDOOR,
    "avia_indoor": AVIA_INDOOR,
    "mid360_outdoor": MID360_OUTDOOR,
    "mid360_indoor": MID360_INDOOR,
}


# ---------------------------------------------------------------------------
# CSV reading
# ---------------------------------------------------------------------------

def read_eigenvalue_ratio(csv_path: Path) -> list[float]:
    """Return list of eigenvalue_ratio values from a diagnostics CSV."""
    values: list[float] = []
    try:
        with csv_path.open() as f:
            reader = csv.DictReader(f)
            for row in reader:
                v = row.get("eigenvalue_ratio")
                if v is None or v == "":
                    continue
                try:
                    fv = float(v)
                except ValueError:
                    continue
                # Skip exact zeros (post-ICDR invalidations per
                # iekf_updater.cpp:978)
                if fv > 0.0:
                    values.append(fv)
    except FileNotFoundError:
        pass
    return values


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def percentile(sorted_values: list[float], p: float) -> float:
    """Linear interpolation percentile from a SORTED list."""
    if not sorted_values:
        return float("nan")
    if p <= 0.0:
        return sorted_values[0]
    if p >= 1.0:
        return sorted_values[-1]
    idx = p * (len(sorted_values) - 1)
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return sorted_values[lo]
    frac = idx - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def summarize(values: list[float]) -> dict:
    if not values:
        return {"n": 0}
    s = sorted(values)
    return {
        "n": len(values),
        "min": s[0],
        "p01": percentile(s, 0.01),
        "p05": percentile(s, 0.05),
        "p10": percentile(s, 0.10),
        "p25": percentile(s, 0.25),
        "p50": percentile(s, 0.50),
        "p75": percentile(s, 0.75),
        "p90": percentile(s, 0.90),
        "p95": percentile(s, 0.95),
        "p99": percentile(s, 0.99),
        "max": s[-1],
        "mean": sum(s) / len(s),
        "geomean_log10": (
            sum(math.log10(v) for v in s if v > 0.0)
            / max(1, sum(1 for v in s if v > 0.0))
        ),
    }


def otsu_log_threshold(
    out_values: list[float], in_values: list[float],
    nbins: int = 200,
) -> float | None:
    """Otsu threshold in log10 space between outdoor and indoor distributions.

    Returns ρ̂_ref such that log10(ρ̂_ref) separates the two pooled
    distributions with maximum between-class variance. Returns None if
    inputs are empty.
    """
    if not out_values or not in_values:
        return None
    log_out = [math.log10(v) for v in out_values if v > 0.0]
    log_in  = [math.log10(v) for v in in_values  if v > 0.0]
    if not log_out or not log_in:
        return None
    all_log = log_out + log_in
    lo, hi = min(all_log), max(all_log)
    if hi - lo < 1e-9:
        return 10.0 ** ((lo + hi) / 2.0)

    # Build histogram
    width = (hi - lo) / nbins
    hist_out = [0] * nbins
    hist_in  = [0] * nbins
    for v in log_out:
        b = min(nbins - 1, int((v - lo) / width))
        hist_out[b] += 1
    for v in log_in:
        b = min(nbins - 1, int((v - lo) / width))
        hist_in[b] += 1

    n_out = sum(hist_out)
    n_in  = sum(hist_in)
    n_tot = n_out + n_in

    # Use combined histogram, treat outdoor as "class 0" if log mean lower,
    # indoor as "class 1" if log mean higher. Otsu max between-class variance.
    combined = [hist_out[i] + hist_in[i] for i in range(nbins)]
    bin_centers = [lo + (i + 0.5) * width for i in range(nbins)]

    total = sum(c * b for c, b in zip(combined, bin_centers))
    w0, sum0, max_var, threshold = 0, 0.0, -1.0, lo
    for i in range(nbins):
        w0 += combined[i]
        if w0 == 0:
            continue
        w1 = n_tot - w0
        if w1 == 0:
            break
        sum0 += combined[i] * bin_centers[i]
        m0 = sum0 / w0
        m1 = (total - sum0) / w1
        var = w0 * w1 * (m0 - m1) ** 2
        if var > max_var:
            max_var = var
            threshold = bin_centers[i]

    return 10.0 ** threshold


def recommend_rho_ref(
    out_stat: dict, in_stat: dict, otsu_val: float | None,
) -> dict:
    """Recommendation triplet: percentile-based, Otsu, geometric mean."""
    rec: dict = {}
    # Percentile-based: p90 of outdoor (severity ≈ 1 above this)
    if out_stat.get("n", 0) > 0:
        rec["p90_out"] = out_stat["p90"]
    # Percentile-based: p10 of indoor (severity ≈ 0 below this)
    if in_stat.get("n", 0) > 0:
        rec["p10_in"] = in_stat["p10"]
    if "p90_out" in rec and "p10_in" in rec:
        rec["geomean_p90_p10"] = math.sqrt(rec["p90_out"] * rec["p10_in"])
    if otsu_val is not None:
        rec["otsu_log10"] = otsu_val
    return rec


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(f"Usage: {argv[0]} <dump_root>", file=sys.stderr)
        return 2
    root = Path(argv[1])
    if not root.exists():
        print(f"ERROR: {root} does not exist", file=sys.stderr)
        return 2

    # Collect per-bucket pooled values
    pooled: dict[str, list[float]] = {k: [] for k in CLASSIFICATION}
    per_seq: dict[str, dict] = {}

    for bucket, seqs in CLASSIFICATION.items():
        for seq in sorted(seqs):
            csv_path = root / seq / "diagnostics.csv"
            vals = read_eigenvalue_ratio(csv_path)
            per_seq[seq] = {
                "bucket": bucket,
                "summary": summarize(vals) if vals else {"n": 0},
            }
            pooled[bucket].extend(vals)

    # Bucket-level summaries
    bucket_summary = {k: summarize(v) for k, v in pooled.items()}

    # Sensor-level recommendations
    avia_out = pooled["avia_outdoor"]
    avia_in  = pooled["avia_indoor"]
    m_out    = pooled["mid360_outdoor"]
    m_in     = pooled["mid360_indoor"]

    otsu_avia   = otsu_log_threshold(avia_out, avia_in)
    otsu_mid360 = otsu_log_threshold(m_out, m_in)

    recommend = {
        "avia": recommend_rho_ref(
            bucket_summary["avia_outdoor"],
            bucket_summary["avia_indoor"],
            otsu_avia,
        ),
        "mid360": recommend_rho_ref(
            bucket_summary["mid360_outdoor"],
            bucket_summary["mid360_indoor"],
            otsu_mid360,
        ),
    }

    summary = {
        "campaign_root": str(root),
        "per_sequence": per_seq,
        "per_bucket": bucket_summary,
        "rho_ref_recommendation": recommend,
        "discriminator_quality": {
            "avia_outdoor_p90_vs_indoor_p10_separation_log10_decades": (
                math.log10(bucket_summary["avia_indoor"]["p10"]) -
                math.log10(bucket_summary["avia_outdoor"]["p90"])
                if (bucket_summary["avia_outdoor"].get("n", 0) > 0 and
                    bucket_summary["avia_indoor"].get("n", 0) > 0 and
                    bucket_summary["avia_outdoor"]["p90"] > 0 and
                    bucket_summary["avia_indoor"]["p10"] > 0)
                else None
            ),
            "mid360_outdoor_p90_vs_indoor_p10_separation_log10_decades": (
                math.log10(bucket_summary["mid360_indoor"]["p10"]) -
                math.log10(bucket_summary["mid360_outdoor"]["p90"])
                if (bucket_summary["mid360_outdoor"].get("n", 0) > 0 and
                    bucket_summary["mid360_indoor"].get("n", 0) > 0 and
                    bucket_summary["mid360_outdoor"]["p90"] > 0 and
                    bucket_summary["mid360_indoor"]["p10"] > 0)
                else None
            ),
        },
    }

    # JSON
    json_path = root / "rho_lambda_summary.json"
    json_path.write_text(json.dumps(summary, indent=2))
    print(f"Wrote {json_path}")

    # Markdown report
    md = ["# S6-R1.2 ρ_λ measurement campaign — analysis report", ""]
    md.append(f"Campaign root: `{root}`")
    md.append("")
    md.append("## Per-bucket summary")
    md.append("")
    md.append("| Bucket | n | p10 | p25 | p50 | p75 | p90 | p99 |")
    md.append("|---|---:|---:|---:|---:|---:|---:|---:|")
    for k, s in bucket_summary.items():
        if s.get("n", 0) == 0:
            md.append(f"| {k} | 0 | — | — | — | — | — | — |")
            continue
        md.append(
            f"| {k} | {s['n']} | "
            f"{s['p10']:.4g} | {s['p25']:.4g} | {s['p50']:.4g} | "
            f"{s['p75']:.4g} | {s['p90']:.4g} | {s['p99']:.4g} |"
        )
    md.append("")
    md.append("## ρ̂_ref recommendation")
    md.append("")
    for sensor, rec in recommend.items():
        md.append(f"### {sensor}")
        for k, v in rec.items():
            md.append(f"- `{k}`: {v:.4g}" if isinstance(v, (int, float)) else f"- `{k}`: {v}")
        md.append("")
    md.append("## Discriminator quality (decades of separation)")
    md.append("")
    for k, v in summary["discriminator_quality"].items():
        md.append(f"- {k}: {v}" if v is not None else f"- {k}: (insufficient data)")

    md_path = root / "rho_lambda_report.md"
    md_path.write_text("\n".join(md) + "\n")
    print(f"Wrote {md_path}")

    # Console summary
    print("\n===== ρ̂_ref RECOMMENDATIONS =====")
    for sensor, rec in recommend.items():
        print(f"  {sensor}:")
        for k, v in rec.items():
            print(f"    {k} = {v}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
