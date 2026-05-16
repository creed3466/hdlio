#!/usr/bin/env python3
"""
verify_gt_quality.py — Compare GT quality between sbs_01 and sbs_03.

Checks for:
- Timestamp monotonicity
- Timestamp gaps (potential Leica tracking loss)
- Position jumps (potential outliers)
- Sampling rate statistics
- Coverage and density

Focus: identify if sbs_03 GT has quality issues during known drift segments
(25-45s, 190-250s elapsed) that could explain ATE discrepancy.
"""

import csv
import sys
import os
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(SCRIPT_DIR)

GT_FILES = {
    "sbs_01": os.path.join(BASE_DIR, "data/ntu_viral/sbs_01/ground_truth.csv"),
    "sbs_03": os.path.join(BASE_DIR, "data/ntu_viral/sbs_03/ground_truth.csv"),
}


def load_gt(path):
    """Load GT: returns arrays of timestamps (sec), positions (Nx3)."""
    times = []
    positions = []
    with open(path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            if len(row) < 10:
                continue
            t = float(row[2]) * 1e-9  # nanoseconds -> seconds
            x, y, z = float(row[3]), float(row[4]), float(row[5])
            times.append(t)
            positions.append([x, y, z])
    return np.array(times), np.array(positions)


def analyze_gt(name, times, positions):
    """Analyze GT quality and return results dict."""
    print(f"\n{'=' * 60}")
    print(f"GT Quality Analysis: {name}")
    print(f"{'=' * 60}")

    n = len(times)
    t0 = times[0]
    duration = times[-1] - times[0]
    print(f"  Points:   {n}")
    print(f"  Duration: {duration:.2f}s")
    print(f"  Start:    {times[0]:.6f}")
    print(f"  End:      {times[-1]:.6f}")

    # Timestamp differences
    dt = np.diff(times)

    # 1. Monotonicity check
    backward = np.sum(dt < 0)
    print(f"\n  --- Monotonicity ---")
    print(f"  Backward jumps: {backward}")
    if backward > 0:
        bad_idx = np.where(dt < 0)[0]
        for idx in bad_idx[:5]:
            print(f"    at idx {idx}: t[{idx}]={times[idx]:.6f} -> t[{idx+1}]={times[idx+1]:.6f} (dt={dt[idx]*1000:.3f}ms)")

    # 2. Sampling rate
    print(f"\n  --- Sampling Rate ---")
    print(f"  Mean dt:   {np.mean(dt)*1000:.3f}ms ({1.0/np.mean(dt):.1f} Hz)")
    print(f"  Median dt: {np.median(dt)*1000:.3f}ms ({1.0/np.median(dt):.1f} Hz)")
    print(f"  Std dt:    {np.std(dt)*1000:.3f}ms")
    print(f"  Min dt:    {np.min(dt)*1000:.3f}ms")
    print(f"  Max dt:    {np.max(dt)*1000:.3f}ms")

    # 3. Timestamp gaps > 0.5s
    print(f"\n  --- Timestamp Gaps (>0.5s) ---")
    gap_mask = dt > 0.5
    n_gaps = np.sum(gap_mask)
    print(f"  Number of gaps >0.5s: {n_gaps}")
    if n_gaps > 0:
        gap_idx = np.where(gap_mask)[0]
        for idx in gap_idx:
            elapsed = times[idx] - t0
            print(f"    at elapsed={elapsed:.1f}s: gap={dt[idx]*1000:.1f}ms ({dt[idx]:.3f}s)")

    # Also check gaps > 0.1s
    gap_mask_100ms = dt > 0.1
    n_gaps_100ms = np.sum(gap_mask_100ms)
    print(f"  Number of gaps >0.1s: {n_gaps_100ms}")
    if n_gaps_100ms > 0:
        gap_idx = np.where(gap_mask_100ms)[0]
        for idx in gap_idx[:10]:
            elapsed = times[idx] - t0
            print(f"    at elapsed={elapsed:.1f}s: gap={dt[idx]*1000:.1f}ms")
        if n_gaps_100ms > 10:
            print(f"    ... ({n_gaps_100ms - 10} more)")

    # 4. Position jumps
    print(f"\n  --- Position Jumps ---")
    dpos = np.diff(positions, axis=0)
    dist = np.linalg.norm(dpos, axis=1)
    velocity = dist / np.maximum(dt, 1e-9)

    print(f"  Position delta stats:")
    print(f"    Mean:   {np.mean(dist)*1000:.3f}mm")
    print(f"    Median: {np.median(dist)*1000:.3f}mm")
    print(f"    Max:    {np.max(dist)*1000:.3f}mm (at elapsed={times[np.argmax(dist)]-t0:.1f}s)")
    print(f"    Std:    {np.std(dist)*1000:.3f}mm")

    # Check for jumps > 0.5m
    jump_mask = dist > 0.5
    n_jumps = np.sum(jump_mask)
    print(f"\n  Position jumps >0.5m: {n_jumps}")
    if n_jumps > 0:
        jump_idx = np.where(jump_mask)[0]
        for idx in jump_idx[:10]:
            elapsed = times[idx] - t0
            print(f"    at elapsed={elapsed:.1f}s: jump={dist[idx]:.3f}m (vel={velocity[idx]:.1f}m/s, dt={dt[idx]*1000:.1f}ms)")

    # Check jumps > 0.1m
    jump_mask_100mm = dist > 0.1
    n_jumps_100mm = np.sum(jump_mask_100mm)
    print(f"  Position jumps >0.1m: {n_jumps_100mm}")

    # 5. Velocity statistics (detect unrealistic motion)
    print(f"\n  --- Velocity Statistics ---")
    # Filter out zero-dt
    valid_vel = velocity[dt > 1e-6]
    print(f"  Mean velocity:   {np.mean(valid_vel):.3f}m/s")
    print(f"  Max velocity:    {np.max(valid_vel):.3f}m/s")
    p99 = np.percentile(valid_vel, 99)
    print(f"  99th percentile: {p99:.3f}m/s")
    unrealistic = np.sum(valid_vel > 5.0)  # >5 m/s is unrealistic for handheld
    print(f"  Points with >5m/s: {unrealistic}")

    # 6. Total travel distance
    total_dist = np.sum(dist)
    print(f"\n  --- Path Statistics ---")
    print(f"  Total travel distance: {total_dist:.2f}m")
    print(f"  Avg speed: {total_dist/duration:.3f}m/s")

    # 7. Focus on drift segments (for sbs_03)
    if name == "sbs_03":
        print(f"\n  --- Drift Segment Analysis ---")
        for seg_name, seg_start, seg_end in [("Segment A", 25, 45), ("Segment B", 190, 250)]:
            elapsed = times - t0
            seg_mask = (elapsed >= seg_start) & (elapsed <= seg_end)
            n_seg = np.sum(seg_mask)
            if n_seg < 2:
                print(f"  {seg_name} ({seg_start}-{seg_end}s): Only {n_seg} GT points!")
                continue

            seg_dt = np.diff(times[seg_mask])
            seg_dist = np.linalg.norm(np.diff(positions[seg_mask], axis=0), axis=1)

            print(f"  {seg_name} ({seg_start}-{seg_end}s):")
            print(f"    GT points:    {n_seg}")
            print(f"    Avg rate:     {1.0/np.mean(seg_dt):.1f}Hz")
            print(f"    Max gap:      {np.max(seg_dt)*1000:.1f}ms")
            print(f"    Max pos jump: {np.max(seg_dist)*1000:.1f}mm")
            gaps_in_seg = np.sum(seg_dt > 0.1)
            print(f"    Gaps >0.1s:   {gaps_in_seg}")

    return {
        'n_points': n,
        'duration': duration,
        'mean_dt_ms': np.mean(dt) * 1000,
        'max_dt_ms': np.max(dt) * 1000,
        'backward_jumps': backward,
        'gaps_500ms': int(np.sum(dt > 0.5)),
        'gaps_100ms': int(np.sum(dt > 0.1)),
        'pos_jumps_500mm': int(np.sum(dist > 0.5)),
        'pos_jumps_100mm': int(np.sum(dist > 0.1)),
        'max_velocity': float(np.max(valid_vel)),
        'total_distance': total_dist,
    }


def main():
    print("=" * 60)
    print("GT Quality Verification: sbs_01 vs sbs_03")
    print("=" * 60)

    results = {}
    for name, path in GT_FILES.items():
        if not os.path.exists(path):
            print(f"\nWARNING: {path} not found, skipping {name}")
            continue
        times, positions = load_gt(path)
        results[name] = analyze_gt(name, times, positions)

    # Comparison
    if len(results) == 2:
        print(f"\n{'=' * 60}")
        print(f"Comparison: sbs_01 vs sbs_03")
        print(f"{'=' * 60}")
        print(f"{'Metric':<25s} {'sbs_01':>15s} {'sbs_03':>15s}")
        print(f"{'-'*55}")
        for key in results['sbs_01']:
            v1 = results['sbs_01'][key]
            v3 = results['sbs_03'][key]
            if isinstance(v1, float):
                print(f"{key:<25s} {v1:>15.3f} {v3:>15.3f}")
            else:
                print(f"{key:<25s} {v1:>15d} {v3:>15d}")

        print(f"\n--- Assessment ---")
        r1 = results['sbs_01']
        r3 = results['sbs_03']
        if r3['gaps_500ms'] > r1['gaps_500ms']:
            print(f"  WARNING: sbs_03 has more large gaps ({r3['gaps_500ms']} vs {r1['gaps_500ms']})")
        if r3['pos_jumps_500mm'] > r1['pos_jumps_500mm']:
            print(f"  WARNING: sbs_03 has more position jumps ({r3['pos_jumps_500mm']} vs {r1['pos_jumps_500mm']})")
        if r3['max_dt_ms'] > 2 * r1['max_dt_ms']:
            print(f"  WARNING: sbs_03 max gap is {r3['max_dt_ms']/r1['max_dt_ms']:.1f}x larger than sbs_01")
        if r3['gaps_500ms'] == 0 and r3['pos_jumps_500mm'] == 0:
            print(f"  sbs_03 GT appears clean: no major gaps or jumps.")
            print(f"  The ~0.40m ATE is likely a real algorithmic limitation, not GT artifact.")


if __name__ == '__main__':
    main()
