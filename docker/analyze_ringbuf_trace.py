#!/usr/bin/env python3
"""Task #70 Proposal A Phase 3 — ringbuf_trace bisection analyzer.

Given a dump directory with per-run `ringbuf_trace_runN.csv` traces, pick one
representative run from Class A and one from Class B, and find the first
`(frame, voxel_key, insertion_index)` at which the two diverge.

Classification of the divergence:
  (a) different `p` (px,py,pz) at the same (frame, voxel_key) ordinal
      → the ingress-order race already altered which point reached the
        ring buffer → M1 CONFIRMED; fix belongs upstream of add_point.
  (b) identical `p` but different `pre_sum` at the same (frame, voxel_key)
      → same input point hit the buffer but the sum differs
      → the FP-addition order on the EARLIER insertions already split;
        M1 is the propagating mechanism but the root is even earlier —
        treat as ingress-order via accumulated earlier points in same voxel.
  (c) identical `p` and identical `pre_sum` but `centroid` differs
      → IEEE-754 impossible (division is deterministic)
      → tooling bug; verify CSV parsing / float round-trip.

Usage:
  python3 docker/analyze_ringbuf_trace.py <DUMP_DIR> \\
      --class-a "1 2 3" --class-b "4 5 6 7 8 9 10"

  <DUMP_DIR> should contain Dark01/runN/ringbuf_trace_runN.csv
"""

import argparse
import csv
import os
import sys
from pathlib import Path


def load_trace(path):
    """Return list of rows as tuples, preserving file order.

    Row = (frame, vkx, vky, vkz, n_before, ring_head_before,
           px, py, pz,
           pre_sx, pre_sy, pre_sz,
           post_sx, post_sy, post_sz,
           cx, cy, cz)
    """
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        # Expected 19 columns (run_id + 18)
        for r in reader:
            if len(r) != 19:
                continue
            rows.append((
                int(r[1]),                 # frame
                int(r[2]), int(r[3]), int(r[4]),  # vkx,vky,vkz
                int(r[5]), int(r[6]),      # n_before, ring_head_before
                float(r[7]), float(r[8]), float(r[9]),   # p
                float(r[10]), float(r[11]), float(r[12]),# pre_sum
                float(r[13]), float(r[14]), float(r[15]),# post_sum
                float(r[16]), float(r[17]), float(r[18]),# centroid
            ))
    return rows


def find_first_divergence(rows_a, rows_b):
    """Walk both traces in lockstep; return (index, reason, row_a, row_b).

    Lockstep is valid only if the ingress ordering is identical up to the
    divergence point. If lengths differ at a given prefix, we treat that as
    an early structural divergence.
    """
    n = min(len(rows_a), len(rows_b))
    for i in range(n):
        ra = rows_a[i]
        rb = rows_b[i]
        if ra == rb:
            continue
        # Dissect the divergence.
        (fa, xa, ya, za, nba, hda,
         pxa, pya, pza,
         psxa, psya, psza,
         qsxa, qsya, qsza,
         cxa, cya, cza) = ra
        (fb, xb, yb, zb, nbb, hdb,
         pxb, pyb, pzb,
         psxb, psyb, pszb,
         qsxb, qsyb, qszb,
         cxb, cyb, czb) = rb

        # Different (frame, voxel) at the same global ordinal = upstream
        # ordering broke before this insertion.
        if (fa, xa, ya, za) != (fb, xb, yb, zb):
            reason = "structural: different (frame, voxel_key) at same global ordinal i"
        elif (pxa, pya, pza) != (pxb, pyb, pzb):
            reason = "cause-a: different p (px,py,pz) at same (frame, voxel)"
        elif (psxa, psya, psza) != (psxb, psyb, pszb):
            reason = "cause-b: identical p but different pre_sum"
        elif (qsxa, qsya, qsza) != (qsxb, qsyb, qszb):
            reason = "cause-b': identical pre_sum but different post_sum (FP-add non-assoc)"
        elif (cxa, cya, cza) != (cxb, cyb, czb):
            reason = "cause-c: identical sums but different centroid (tooling bug)"
        else:
            reason = "unknown: tuple mismatch without field-level difference"
        return i, reason, ra, rb

    if len(rows_a) != len(rows_b):
        return (n, f"length mismatch: |A|={len(rows_a)} |B|={len(rows_b)}",
                rows_a[n] if n < len(rows_a) else None,
                rows_b[n] if n < len(rows_b) else None)
    return None


def verdict(reason):
    if reason.startswith("cause-a"):
        return "CONFIRM-M1 (ingress-order race reached ring_buffer)"
    if reason.startswith("cause-b"):
        return ("CONFIRM-M1 (FP-add non-associativity inside ring buffer) — "
                "Proposal B-a applicable")
    if reason.startswith("cause-c"):
        return "TOOLING-BUG"
    if reason.startswith("structural") or reason.startswith("length"):
        return ("UPSTREAM-DIVERGENCE (ordering broke before ring_buffer) — "
                "M1 inconclusive; look earlier in the pipeline")
    return "INCONCLUSIVE"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump_dir", help="Base dump dir containing Dark01/runN/")
    ap.add_argument("--class-a", required=True,
                    help="space-separated run IDs for Class A")
    ap.add_argument("--class-b", required=True,
                    help="space-separated run IDs for Class B")
    ap.add_argument("--seq", default="Dark01")
    args = ap.parse_args()

    runs_a = [int(x) for x in args.class_a.split() if x.strip()]
    runs_b = [int(x) for x in args.class_b.split() if x.strip()]
    if not runs_a or not runs_b:
        print("ERROR: both --class-a and --class-b must be non-empty.",
              file=sys.stderr)
        sys.exit(2)

    ra_id, rb_id = runs_a[0], runs_b[0]
    base = Path(args.dump_dir)
    path_a = base / args.seq / f"run{ra_id}" / f"ringbuf_trace_run{ra_id}.csv"
    path_b = base / args.seq / f"run{rb_id}" / f"ringbuf_trace_run{rb_id}.csv"

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
        print("That contradicts the class split; inspect traces manually.")
        sys.exit(4)

    idx, reason, ra, rb = result
    print(f"\n==== First divergence @ global insertion ordinal i={idx} ====")
    print(f"Reason: {reason}")
    if ra is not None:
        print(f"  A row: {ra}")
    if rb is not None:
        print(f"  B row: {rb}")
    print(f"\nVerdict: {verdict(reason)}")


if __name__ == "__main__":
    main()
