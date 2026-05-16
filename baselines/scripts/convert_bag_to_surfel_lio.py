#!/usr/bin/env python3
"""Convert M3DGR rosbag (livox_ros_driver2/CustomMsg) to Surfel-LIO format.

Output format:
  <output_dir>/
    imu_data.csv         — timestamp,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z
    lidar_timestamps.txt — one timestamp per scan line
    lidar/000000.ply     — binary little-endian PLY (x,y,z float + intensity float + offset_time_ns uint32)

Usage:
  python3 convert_bag_to_surfel_lio.py <bag_path> <output_dir> [--imu-topic /livox/mid360/imu] [--lidar-topic /livox/mid360/lidar]
"""
import argparse
import os
import struct
import sys
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description="Convert rosbag to Surfel-LIO format")
    parser.add_argument("bag_path", help="Path to .bag file")
    parser.add_argument("output_dir", help="Output directory")
    parser.add_argument("--imu-topic", default="/livox/mid360/imu", help="IMU topic")
    parser.add_argument("--lidar-topic", default="/livox/mid360/lidar", help="LiDAR topic")
    args = parser.parse_args()

    try:
        import rosbag
    except ImportError:
        print("[ERROR] rosbag not found. Install with: pip install rosbag --extra-index-url https://rospkg.github.io/simple/")
        sys.exit(1)

    bag_path = args.bag_path
    out_dir = Path(args.output_dir)
    lidar_dir = out_dir / "lidar"
    lidar_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Converting {bag_path} → {out_dir}")
    print(f"[INFO] IMU topic: {args.imu_topic}, LiDAR topic: {args.lidar_topic}")

    bag = rosbag.Bag(bag_path, 'r')

    # --- IMU ---
    # M3DGR Mid360 bags publish acceleration in g-units; scale to m/s².
    G = 9.80665
    imu_count = 0
    with open(out_dir / "imu_data.csv", "w") as f:
        f.write("timestamp,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z\n")
        for topic, msg, t in bag.read_messages(topics=[args.imu_topic]):
            ts = msg.header.stamp.to_sec()
            gx = msg.angular_velocity.x
            gy = msg.angular_velocity.y
            gz = msg.angular_velocity.z
            ax = msg.linear_acceleration.x * G
            ay = msg.linear_acceleration.y * G
            az = msg.linear_acceleration.z * G
            f.write(f"{ts:.9f},{gx},{gy},{gz},{ax},{ay},{az}\n")
            imu_count += 1

    print(f"[INFO] IMU: {imu_count} samples")

    # --- LiDAR (livox_ros_driver2/CustomMsg) ---
    scan_index = 0
    ts_file = open(out_dir / "lidar_timestamps.txt", "w")

    for topic, msg, t in bag.read_messages(topics=[args.lidar_topic]):
        # CustomMsg structure:
        #   header (std_msgs/Header)
        #   timebase (uint64)
        #   point_num (uint32)
        #   lidar_id (uint8)
        #   rsvd[3] (uint8)
        #   points[] (CustomPoint: offset_time uint32, x/y/z float32, reflectivity uint8, tag uint8, line uint8)

        scan_ts = msg.header.stamp.to_sec()
        ts_file.write(f"{scan_ts:.9f}\n")

        # Write PLY
        points = msg.points
        n_pts = len(points)
        ply_path = lidar_dir / f"{scan_index:06d}.ply"

        with open(ply_path, "wb") as f:
            # ASCII header
            header = (
                "ply\n"
                "format binary_little_endian 1.0\n"
                f"element vertex {n_pts}\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "property float intensity\n"
                "property uint offset_time_ns\n"
                "end_header\n"
            )
            f.write(header.encode('ascii'))

            # Binary data: x(f), y(f), z(f), intensity(f), offset_time_ns(I)
            for pt in points:
                f.write(struct.pack('<ffffi',
                    pt.x, pt.y, pt.z,
                    float(pt.reflectivity),
                    pt.offset_time))  # offset_time already in nanoseconds

        scan_index += 1
        if scan_index % 100 == 0:
            print(f"  [{scan_index}] scans processed...")

    ts_file.close()
    bag.close()

    print(f"[INFO] LiDAR: {scan_index} scans → {lidar_dir}")
    print(f"[DONE] Conversion complete: {out_dir}")

if __name__ == "__main__":
    main()
