#!/usr/bin/env python3
"""
Sprint 14 B.R0.4.1 — Phase 1 empirical separation gate extractor.

Reads an Avia rosbag, accumulates first N IMU samples + first LiDAR scan,
computes 4 predicate clauses for the pre-frame-0 CLASS_D classifier hook:
  planarity = 1 - sigma1^2 / sigma3^2   (PCA on raw scan points)
  degen_count = count of sigma_i^2/sigma_3^2 < tau   (default tau=0.001)
  eigvec_min_world_z_abs = |dot(eigvec_of_smallest_eigenvalue, world_z)|
  g_residual_norm = fabs(||mean(accel)|| - 9.81)   (m/s^2)

Acceptance margins (architect R0.4 §8.1):
  PASS clause:
    planarity         >= 0.55  (R-A 0.50 + 10% buffer)
    degen_count       <= 1     (strict)
    eigvec_min_z_abs  >= 0.7   (cos(45°))
    g_residual_norm   <= 0.18  (2σ IMU noise design ceiling - 10% buffer)

Outputs CSV row to <out_csv>.

Usage:
  python3 preframe0_extractor.py \
    --bag <bag.bag> \
    --imu-topic /livox/avia/imu \
    --lidar-topic /livox/avia/lidar \
    --seq-name VI03 \
    --out docs/experiments/preframe0_classifier_features.csv \
    [--n-imu 100]

Architecture: pure read-only. No state mutation. No spdlog/logging dependency.
Designed for execution inside the tofslam:ros1 Docker container (which has
the livox_ros_driver Python message bindings + rosbag library).
"""
import argparse
import csv
import math
import os
import sys
from pathlib import Path

import numpy as np


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True, help="Input rosbag path")
    p.add_argument("--imu-topic", default="/livox/avia/imu")
    p.add_argument("--lidar-topic", default="/livox/avia/lidar")
    p.add_argument("--seq-name", required=True)
    p.add_argument("--out", required=True, help="Append CSV row to this file")
    p.add_argument("--n-imu", type=int, default=100)
    p.add_argument(
        "--degen-tau",
        type=float,
        default=0.001,
        help="Eigenvalue ratio threshold for degeneracy count (default 1e-3)",
    )
    return p.parse_args()


def extract_lidar_points(msg):
    """Extract (N, 3) array of points from livox CustomMsg or PointCloud2."""
    # Try livox CustomMsg first (Avia)
    if hasattr(msg, "points") and len(msg.points) > 0 and hasattr(msg.points[0], "line"):
        pts = np.array(
            [[p.x, p.y, p.z] for p in msg.points],
            dtype=np.float32,
        )
        return pts
    # Fallback: PointCloud2
    if hasattr(msg, "data") and hasattr(msg, "point_step"):
        import sensor_msgs.point_cloud2 as pc2
        pts = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32,
        )
        return pts
    raise RuntimeError(f"Unknown LiDAR message type: {type(msg).__name__}")


def main():
    args = parse_args()

    # Import rosbag lazily (only available inside Docker container)
    import rosbag
    import rospy

    bag_path = Path(args.bag)
    if not bag_path.exists():
        sys.stderr.write(f"[extractor] ERROR: bag not found: {bag_path}\n")
        sys.exit(2)

    imu_accel = []  # list of (ax, ay, az)
    imu_gyro = []   # list of (gx, gy, gz)
    first_scan_pts = None

    sys.stderr.write(f"[extractor] {args.seq_name}: opening bag {bag_path}\n")
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[args.imu_topic, args.lidar_topic]):
            if topic == args.imu_topic and len(imu_accel) < args.n_imu:
                a = msg.linear_acceleration
                g = msg.angular_velocity
                imu_accel.append([a.x, a.y, a.z])
                imu_gyro.append([g.x, g.y, g.z])
            elif topic == args.lidar_topic and first_scan_pts is None:
                first_scan_pts = extract_lidar_points(msg)
            # Early exit once both gathered
            if first_scan_pts is not None and len(imu_accel) >= args.n_imu:
                break

    if first_scan_pts is None:
        sys.stderr.write(f"[extractor] {args.seq_name}: ERROR no LiDAR found\n")
        sys.exit(3)
    if len(imu_accel) < args.n_imu:
        sys.stderr.write(
            f"[extractor] {args.seq_name}: WARN only {len(imu_accel)}/{args.n_imu} IMU samples (using all)\n"
        )

    imu_accel = np.asarray(imu_accel, dtype=np.float64)
    imu_gyro = np.asarray(imu_gyro, dtype=np.float64)

    # --- IMU features ---
    mean_accel = imu_accel.mean(axis=0)
    g_residual_norm = float(abs(np.linalg.norm(mean_accel) - 9.81))
    accel_std = float(np.linalg.norm(imu_accel, axis=1).std())
    gyro_std = float(np.linalg.norm(imu_gyro, axis=1).std())

    # --- LiDAR PCA ---
    # Filter invalid / zero points (raw Avia includes zero-valued holes)
    valid = np.linalg.norm(first_scan_pts, axis=1) > 0.1
    pts = first_scan_pts[valid].astype(np.float64)
    n_raw = int(first_scan_pts.shape[0])
    n_valid = int(pts.shape[0])

    if n_valid < 10:
        sys.stderr.write(f"[extractor] {args.seq_name}: ERROR n_valid={n_valid} insufficient\n")
        sys.exit(4)

    centroid = pts.mean(axis=0)
    centered = pts - centroid
    cov = centered.T @ centered / float(n_valid)
    # eigenvalues sorted ascending → sigma1^2 (smallest) ... sigma3^2 (largest)
    eigvals, eigvecs = np.linalg.eigh(cov)
    sigma1_sq, sigma2_sq, sigma3_sq = float(eigvals[0]), float(eigvals[1]), float(eigvals[2])
    eigvec_smallest = eigvecs[:, 0]  # column 0 = smallest eigenvalue eigenvector

    # planarity in [0, 1]: 1 - sigma1^2 / sigma3^2
    planarity = 0.0
    if sigma3_sq > 1e-12:
        planarity = float(1.0 - sigma1_sq / sigma3_sq)

    # degen_count: how many eigenvalue RATIOs to max are below tau
    degen_count = int(
        sum(1 for i in range(3) if (eigvals[i] / sigma3_sq) < args.degen_tau)
    )

    # eigvec_min_world_z_abs: |smallest_eigvec . world_z|
    world_z = np.array([0.0, 0.0, 1.0])
    eigvec_min_world_z_abs = float(abs(eigvec_smallest @ world_z))

    # --- Predicate clauses ---
    PLANARITY_THR = 0.55
    DEGEN_THR = 1  # <= 1
    EIGVEC_Z_THR = 0.7
    G_RES_THR = 0.18

    c_planarity = planarity >= PLANARITY_THR
    c_degen = degen_count <= DEGEN_THR
    c_eigvec_z = eigvec_min_world_z_abs >= EIGVEC_Z_THR
    c_g_residual = g_residual_norm <= G_RES_THR

    predicate_all = c_planarity and c_degen and c_eigvec_z and c_g_residual

    # --- Margin computation (signed % relative to threshold) ---
    def margin(value, thr, direction):
        # direction='ge' = pass when value >= thr; margin = (value - thr) / thr
        # direction='le' = pass when value <= thr; margin = (thr - value) / thr
        if thr == 0:
            return 0.0
        if direction == "ge":
            return (value - thr) / abs(thr)
        else:
            return (thr - value) / abs(thr)

    margin_planarity = margin(planarity, PLANARITY_THR, "ge")
    # degen threshold is integer, use signed difference normalized
    margin_degen = float(DEGEN_THR - degen_count)
    margin_eigvec_z = margin(eigvec_min_world_z_abs, EIGVEC_Z_THR, "ge")
    margin_g_res = margin(g_residual_norm, G_RES_THR, "le")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "seq",
        "n_raw",
        "n_valid",
        "sigma1_sq",
        "sigma2_sq",
        "sigma3_sq",
        "planarity",
        "degen_count",
        "eigvec_min_world_z_abs",
        "g_residual_norm",
        "accel_std",
        "gyro_std",
        "n_imu",
        "c_planarity",
        "c_degen",
        "c_eigvec_z",
        "c_g_residual",
        "predicate_all",
        "margin_planarity",
        "margin_degen",
        "margin_eigvec_z",
        "margin_g_res",
    ]
    row = [
        args.seq_name,
        n_raw,
        n_valid,
        sigma1_sq,
        sigma2_sq,
        sigma3_sq,
        planarity,
        degen_count,
        eigvec_min_world_z_abs,
        g_residual_norm,
        accel_std,
        gyro_std,
        int(len(imu_accel)),
        int(c_planarity),
        int(c_degen),
        int(c_eigvec_z),
        int(c_g_residual),
        int(predicate_all),
        margin_planarity,
        margin_degen,
        margin_eigvec_z,
        margin_g_res,
    ]
    write_header = not out_path.exists()
    with open(out_path, "a", newline="") as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(header)
        w.writerow(row)
    sys.stderr.write(
        f"[extractor] {args.seq_name}: planarity={planarity:.4f} degen={degen_count} "
        f"eigvec_z={eigvec_min_world_z_abs:.4f} g_res={g_residual_norm:.4f} "
        f"predicate_all={predicate_all}\n"
    )


if __name__ == "__main__":
    main()
