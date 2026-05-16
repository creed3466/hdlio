#!/usr/bin/env python3
"""Convert NTU VIRAL ground_truth.csv -> TUM format trajectory.

Input CSV columns (NTU VIRAL standard):
  %time, field.header.seq, field.header.stamp,
  field.pose.position.{x,y,z},
  field.pose.orientation.{x,y,z,w}

TUM format:
  timestamp_sec tx ty tz qx qy qz qw

Timestamp source: field.header.stamp (ns) -> seconds.
"""

from __future__ import annotations
import argparse
import csv
from pathlib import Path


def convert(src: Path, dst: Path) -> int:
    n = 0
    with src.open() as f_in, dst.open("w") as f_out:
        reader = csv.DictReader(f_in)
        for row in reader:
            t_ns = int(row["field.header.stamp"])
            t_s = t_ns / 1e9
            tx = row["field.pose.position.x"]
            ty = row["field.pose.position.y"]
            tz = row["field.pose.position.z"]
            qx = row["field.pose.orientation.x"]
            qy = row["field.pose.orientation.y"]
            qz = row["field.pose.orientation.z"]
            qw = row["field.pose.orientation.w"]
            f_out.write(f"{t_s:.9f} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
            n += 1
    return n


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("csv", type=Path, help="input NTU ground_truth.csv")
    p.add_argument("tum", type=Path, help="output TUM trajectory")
    args = p.parse_args()
    n = convert(args.csv, args.tum)
    print(f"[ok] {args.csv.name} -> {args.tum} ({n} poses)")


if __name__ == "__main__":
    main()
