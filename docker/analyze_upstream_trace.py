#!/usr/bin/env python3
"""Task #70 U1 upstream bisection analyzer.

Given per-run ``upstream_trace_runN.csv`` traces (emitted by
``tof_slam/frontend/diag/upstream_trace.hpp`` at map-update time), pick one
representative run from Class A and one from Class B, walk both traces in
lockstep, and classify the first divergence.

Schema per row (CSV with header):
  run_id, frame, idx_in_frame,
  bx, by, bz,        # body-frame point handed to transform_to_world()
  wx, wy, wz,        # world-frame point returned
  sx, sy, sz,        # state_.position() at that frame's map-update
  qx, qy, qz, qw     # state_.pose().rotation as quaternion

Classification:
  U1a  bp (bx,by,bz) differs at same (frame, idx_in_frame)
       → preprocess / IMU undistortion race upstream of IEKF.
  U1b  bp identical, state (s + q) differs
       → IEKF inner-iteration race (order of fusion varies).
  U1c  bp + state identical, wp differs
       → IEEE-754 impossible under single-thread scalar Eigen;
       tooling bug (CSV parsing / float round-trip).
  STRUCT  frame or idx_in_frame diverges at the same global ordinal
          → per-frame point count already diverged (upstream filter/undistort
          dropped different counts per run).

Usage:
  python3 docker/analyze_upstream_trace.py <DUMP_SEQ_DIR> \\
      --class-a "1 2 3" --class-b "4 5 6 7"

  <DUMP_SEQ_DIR> is expected to contain runN/upstream_trace_runN.csv.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Tuple


Row = Tuple[int, int, float, float, float, float, float, float,
            float, float, float, float, float, float, float]


def load_trace(path: Path) -> list[Row]:
    rows: list[Row] = []
    with path.open("r", newline="") as fp:
        reader = csv.reader(fp)
        next(reader, None)  # header
        for r in reader:
            if len(r) != 16:
                continue
            rows.append((
                int(r[1]),                           # frame
                int(r[2]),                           # idx_in_frame
                float(r[3]), float(r[4]), float(r[5]),   # bp
                float(r[6]), float(r[7]), float(r[8]),   # wp
                float(r[9]), float(r[10]), float(r[11]), # state pos
                float(r[12]), float(r[13]), float(r[14]), float(r[15]),  # quat xyzw
            ))
    return rows


def classify(ra: Row, rb: Row) -> str:
    (fa, ia, bxa, bya, bza, wxa, wya, wza,
     sxa, sya, sza, qxa, qya, qza, qwa) = ra
    (fb, ib, bxb, byb, bzb, wxb, wyb, wzb,
     sxb, syb, szb, qxb, qyb, qzb, qwb) = rb

    if (fa, ia) != (fb, ib):
        return ("STRUCT: frame/idx_in_frame diverges at same global ordinal — "
                "point count upstream of this hook already diverged")

    if (bxa, bya, bza) != (bxb, byb, bzb):
        return ("U1a: bp differs at same (frame, idx_in_frame) "
                "→ preprocess / IMU undistortion race")

    state_a = (sxa, sya, sza, qxa, qya, qza, qwa)
    state_b = (sxb, syb, szb, qxb, qyb, qzb, qwb)
    if state_a != state_b:
        return ("U1b: bp identical, state (pos+quat) differs "
                "→ IEKF inner-iteration race")

    if (wxa, wya, wza) != (wxb, wyb, wzb):
        return ("U1c: bp + state identical, wp differs "
                "→ IEEE-754 impossible; tooling bug")

    return "UNKNOWN: tuples differ but no field-level difference"


def verdict(reason: str) -> str:
    if reason.startswith("U1a"):
        return "U1a CONFIRMED — fix belongs in preprocess / IMU undistortion."
    if reason.startswith("U1b"):
        return "U1b CONFIRMED — fix belongs in IEKF inner-iteration order."
    if reason.startswith("U1c"):
        return ("U1c — IEEE-754 impossible; investigate CSV tooling and "
                "per-point float round-trip.")
    if reason.startswith("STRUCT"):
        return ("STRUCTURAL DIVERGENCE — look even earlier than "
                "transform_to_world (filter / undistort point counts differ).")
    return "INCONCLUSIVE"


def find_first_divergence(rows_a: list[Row], rows_b: list[Row]):
    n = min(len(rows_a), len(rows_b))
    for i in range(n):
        if rows_a[i] != rows_b[i]:
            return i, classify(rows_a[i], rows_b[i]), rows_a[i], rows_b[i]
    if len(rows_a) != len(rows_b):
        return (n,
                f"LENGTH: |A|={len(rows_a)} |B|={len(rows_b)}",
                rows_a[n] if n < len(rows_a) else None,
                rows_b[n] if n < len(rows_b) else None)
    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("dump_seq_dir",
                    help="Dump seq dir containing runN/upstream_trace_runN.csv")
    ap.add_argument("--class-a", required=True,
                    help="space-separated run IDs for Class A")
    ap.add_argument("--class-b", required=True,
                    help="space-separated run IDs for Class B")
    args = ap.parse_args()

    runs_a = [int(x) for x in args.class_a.split() if x.strip()]
    runs_b = [int(x) for x in args.class_b.split() if x.strip()]
    if not runs_a or not runs_b:
        print("ERROR: class-a and class-b must be non-empty.", file=sys.stderr)
        sys.exit(2)

    base = Path(args.dump_seq_dir)
    ra_id, rb_id = runs_a[0], runs_b[0]
    path_a = base / f"run{ra_id}" / f"upstream_trace_run{ra_id}.csv"
    path_b = base / f"run{rb_id}" / f"upstream_trace_run{rb_id}.csv"

    for p in (path_a, path_b):
        if not p.exists():
            print(f"ERROR: missing {p}", file=sys.stderr)
            sys.exit(3)

    print(f"Class A representative: run{ra_id} ({path_a})")
    print(f"Class B representative: run{rb_id} ({path_b})")
    rows_a = load_trace(path_a)
    rows_b = load_trace(path_b)
    print(f"Rows: |A|={len(rows_a)} |B|={len(rows_b)}")

    result = find_first_divergence(rows_a, rows_b)
    if result is None:
        print("\nNo divergence found — both traces bit-identical.")
        print("That contradicts the ATE class split; inspect traces manually.")
        sys.exit(4)

    idx, reason, ra, rb = result
    print(f"\n==== First divergence @ global ordinal i={idx} ====")
    print(f"Reason: {reason}")
    if ra is not None:
        print(f"  A row: {ra}")
    if rb is not None:
        print(f"  B row: {rb}")
    print(f"\nVerdict: {verdict(reason)}")


if __name__ == "__main__":
    main()
