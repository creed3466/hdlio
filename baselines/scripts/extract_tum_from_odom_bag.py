#!/usr/bin/env python3
"""extract_tum_from_odom_bag.py — Extract TUM-format trajectory from a rosbag.

Reads nav_msgs/Odometry messages and writes:
    timestamp x y z qx qy qz qw
one per line (space-separated — TUM format).

Usage:
    extract_tum_from_odom_bag.py <bag> <topic> <out_csv>
"""
from __future__ import annotations

import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) < 4:
        print(__doc__, file=sys.stderr)
        return 2

    bag_path = Path(sys.argv[1])
    topic = sys.argv[2]
    out_path = Path(sys.argv[3])

    if not bag_path.exists():
        print(f"[ERROR] Bag not found: {bag_path}", file=sys.stderr)
        return 1

    try:
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        print("[ERROR] rosbags package not available. Install: pip install rosbags", file=sys.stderr)
        return 1

    typestore = get_typestore(Stores.ROS1_NOETIC)
    count = 0
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with Reader(bag_path) as reader, out_path.open("w") as fout:
        fout.write("# timestamp tx ty tz qx qy qz qw\n")
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            available = sorted({c.topic for c in reader.connections})
            print(f"[ERROR] Topic {topic} not found. Available: {available}", file=sys.stderr)
            return 1
        for conn, ts, raw in reader.messages(connections=conns):
            msg = typestore.deserialize_ros1(raw, conn.msgtype)
            stamp = msg.header.stamp
            t = stamp.sec + stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            fout.write(f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
                       f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
            count += 1

    print(f"[OK] Wrote {count} poses to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
