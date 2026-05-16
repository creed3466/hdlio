#!/usr/bin/env python3
"""Parse timing results from baseline runs and TofSLAM paper dump.

Reads:
  dump/timing_baselines/fast_lio2_timing.csv   (per-scan CSV)
  dump/timing_baselines/fast_lio2_stdout.log   (printf output, running avg)
  dump/timing_baselines/point_lio_stdout.log   (printf output, running avg)
  dump/timing_baselines/ig_lio_stdout.log      (glog output, Timer::PrintAll)
  dump/paper_20260418/avia/*/timing.csv        (TofSLAM per-scan CSV)

Outputs summary for paper table.
"""
import csv
import glob
import os
import re
import statistics
import sys


def parse_fast_lio2_csv(path: str) -> dict:
    """Parse FAST-LIO2 CSV (per-scan total time column)."""
    totals = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            totals.append(float(row[" total time"]))
    if not totals:
        return {}
    totals_ms = [t * 1000 for t in totals]  # seconds → ms
    totals_ms.sort()
    return {
        "mean": statistics.mean(totals_ms),
        "p95": totals_ms[int(len(totals_ms) * 0.95)],
        "frames": len(totals_ms),
    }


def parse_ave_total_from_stdout(path: str) -> dict:
    """Parse 'ave total' from printf stdout log (last line is final average)."""
    last_total = None
    frame_count = 0
    all_per_frame = []
    pattern = re.compile(r"ave total:\s*([0-9.]+)")
    # Also try to get per-frame times from the individual columns
    per_frame_pattern = re.compile(
        r"IMU \+ Map \+ Input Downsample:\s*([0-9.]+).*?ave total:\s*([0-9.]+)"
    )
    with open(path) as f:
        for line in f:
            m = pattern.search(line)
            if m:
                last_total = float(m.group(1))
                frame_count += 1
    if last_total is None:
        return {}
    return {
        "ave_total_s": last_total,
        "ave_total_ms": last_total * 1000,
        "frames": frame_count,
    }


def parse_ig_lio_stdout(path: str) -> dict:
    """Parse iG-LIO Timer::PrintAll() glog output."""
    results = {}
    pattern = re.compile(
        r"\[ (.+?) \] average time usage:\s*([0-9.]+)\s*ms.*called times:\s*(\d+)"
    )
    with open(path) as f:
        for line in f:
            m = pattern.search(line)
            if m:
                name = m.group(1).strip()
                avg_ms = float(m.group(2))
                count = int(m.group(3))
                results[name] = {"avg_ms": avg_ms, "count": count}
    return results


def parse_tofslam_timing(paper_dir: str) -> dict:
    """Parse TofSLAM timing from paper dump CSVs."""
    all_totals = []
    for seq_path in sorted(glob.glob(os.path.join(paper_dir, "*/timing.csv"))):
        with open(seq_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                all_totals.append(float(row["total_ms"]))
    if not all_totals:
        return {}
    all_totals.sort()
    return {
        "mean": statistics.mean(all_totals),
        "p95": all_totals[int(len(all_totals) * 0.95)],
        "frames": len(all_totals),
    }


def main():
    base = "dump/timing_baselines"

    print("=" * 60)
    print("TIMING COMPARISON — Dark01 sequence, i7-12700, single-threaded")
    print("=" * 60)
    print()

    # TofSLAM (all 9 Avia sequences)
    ts = parse_tofslam_timing("dump/paper_20260418/avia")
    if ts:
        print(f"TofSLAM (9 seqs):  mean={ts['mean']:.1f} ms, p95={ts['p95']:.1f} ms  ({ts['frames']} frames)")

    # FAST-LIO2 CSV
    flio_csv = os.path.join(base, "fast_lio2_timing.csv")
    if os.path.exists(flio_csv) and os.path.getsize(flio_csv) > 200:
        fl = parse_fast_lio2_csv(flio_csv)
        if fl:
            print(f"FAST-LIO2 (CSV):   mean={fl['mean']:.1f} ms, p95={fl['p95']:.1f} ms  ({fl['frames']} frames)")

    # FAST-LIO2 stdout
    flio_log = os.path.join(base, "fast_lio2_stdout.log")
    if os.path.exists(flio_log):
        fl2 = parse_ave_total_from_stdout(flio_log)
        if fl2:
            print(f"FAST-LIO2 (avg):   ave_total={fl2['ave_total_ms']:.1f} ms  ({fl2['frames']} lines)")

    # Point-LIO stdout
    plio_log = os.path.join(base, "point_lio_stdout.log")
    if os.path.exists(plio_log) and os.path.getsize(plio_log) > 0:
        pl = parse_ave_total_from_stdout(plio_log)
        if pl:
            print(f"Point-LIO (avg):   ave_total={pl['ave_total_ms']:.1f} ms  ({pl['frames']} lines)")

    # iG-LIO stdout
    iglio_log = os.path.join(base, "ig_lio_stdout.log")
    if os.path.exists(iglio_log):
        ig = parse_ig_lio_stdout(iglio_log)
        if ig:
            print("iG-LIO timers:")
            for name, vals in sorted(ig.items()):
                print(f"  {name}: {vals['avg_ms']:.1f} ms (n={vals['count']})")
        else:
            print("iG-LIO: no timer entries found")

    print()
    print("=" * 60)


if __name__ == "__main__":
    main()
