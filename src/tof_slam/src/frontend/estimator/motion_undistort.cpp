// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// motion_undistort.cpp — LUT-based motion undistortion (matches reference).
//
// Pre-computes N relative transforms, then uses O(1) lookup per point.
// This is ~100x faster than per-point SE(3) interpolation.

#include "tof_slam/frontend/estimator/motion_undistort.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// interpolate_pose
// ---------------------------------------------------------------------------

Se3 interpolate_pose(const Se3& p0, const Se3& p1, float alpha) {
  const Se3 delta = p0.inverse() * p1;
  const Eigen::Matrix<float, 6, 1> xi = delta.Log() * alpha;
  return p0 * Se3::Exp(xi);
}

// ---------------------------------------------------------------------------
// Helper: find index of the last trajectory entry with timestamp <= t
// ---------------------------------------------------------------------------
static std::size_t lower_bound_idx(const std::vector<StampedPose>& traj,
                                    double t) {
  std::size_t lo = 0;
  std::size_t hi = traj.size();
  while (lo + 1 < hi) {
    const std::size_t mid = lo + (hi - lo) / 2u;
    if (traj[mid].timestamp <= t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

// ---------------------------------------------------------------------------
// interpolate_at — returns interpolated pose at time t
// ---------------------------------------------------------------------------
static Se3 interpolate_at(const std::vector<StampedPose>& traj, double t) {
  if (t <= traj.front().timestamp) {
    return traj.front().pose;
  }
  if (t >= traj.back().timestamp) {
    return traj.back().pose;
  }

  const std::size_t lo_idx = lower_bound_idx(traj, t);
  const std::size_t hi_idx = lo_idx + 1u;

  const double dt = traj[hi_idx].timestamp - traj[lo_idx].timestamp;

  float alpha = 0.0f;
  if (dt > 1e-12) {
    alpha = static_cast<float>((t - traj[lo_idx].timestamp) / dt);
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
  }

  return interpolate_pose(traj[lo_idx].pose, traj[hi_idx].pose, alpha);
}

// ---------------------------------------------------------------------------
// undistort_scan — LUT-based (matches reference Estimator.cpp lines 1120-1213)
// ---------------------------------------------------------------------------

PointCloud undistort_scan(const PointCloud& cloud,
                          const std::vector<StampedPose>& trajectory,
                          double scan_end_time,
                          const Se3& T_body_lidar) {
  if (trajectory.size() < 2u) {
    return cloud;
  }

  // --- 1. Determine scan time window ---
  // offset_time = absolute_point_time - message_header_time (positive, from
  // scan start).  scan_end_time is the message header timestamp, which for
  // Livox MID-360 is the scan START time.
  //
  // Reference frame for undistortion = scan_end_time (= scan start).  This
  // matches the IEKF, which uses the state propagated to scan_end_time.
  float min_offset = 0.0f;
  float max_offset = 0.0f;
  for (const auto& pt : cloud) {
    if (pt.offset_time < min_offset) min_offset = pt.offset_time;
    if (pt.offset_time > max_offset) max_offset = pt.offset_time;
  }

  const double scan_duration = static_cast<double>(max_offset - min_offset);

  if (scan_duration < 1e-6) {
    return cloud;  // All points at same time — nothing to undistort.
  }

  // --- 2. Pre-compute N relative transforms (LUT) ---
  // Reference uses N=100 segments. This avoids per-point SE3 Log/Exp.
  constexpr int N = 100;
  const double dt_seg = scan_duration / N;

  // Absolute time of first and last point
  const double t_first = scan_end_time + static_cast<double>(min_offset);
  const double t_last  = scan_end_time + static_cast<double>(max_offset);

  // Extrinsic decomposition
  const Eigen::Matrix3f R_bl = T_body_lidar.rotation_matrix();  // R_body_lidar
  const Eigen::Vector3f t_bl = T_body_lidar.translation();       // t_body_lidar
  const Eigen::Matrix3f R_bl_T = R_bl.transpose();               // R_lidar_body

  // Reference pose = pose at scan_end_time (message header = scan start).
  // All points are corrected to this frame, matching the IEKF state.
  const Se3 T_ref = interpolate_at(trajectory, scan_end_time);
  const Eigen::Matrix3f R_ref = T_ref.rotation_matrix();
  const Eigen::Vector3f t_ref = T_ref.translation();
  const Eigen::Matrix3f R_ref_T = R_ref.transpose();

  // Pre-compute relative rotation and translation for each segment.
  // Transform: p_lidar_ref = R_rel[k] * p_lidar_k + t_rel[k]
  //
  // Chain: LiDAR_k → body_k → world → body_ref → LiDAR_ref
  //   p_body_k     = R_bl * p_lidar + t_bl
  //   p_world      = R_k * p_body_k + t_k
  //   p_body_ref   = R_ref^T * (p_world - t_ref)
  //   p_lidar_ref  = R_bl^T * (p_body_ref - t_bl)
  //
  // Combining:
  //   R_rel = R_bl^T * R_ref^T * R_k * R_bl
  //   t_rel = R_bl^T * R_ref^T * (R_k * t_bl + t_k - t_ref) - R_bl^T * t_bl

  std::vector<Eigen::Matrix3f> R_rel(N);
  std::vector<Eigen::Vector3f> t_rel(N);

  for (int k = 0; k < N; ++k) {
    const double t_k = t_first + k * dt_seg;
    const Se3 T_k = interpolate_at(trajectory, t_k);
    const Eigen::Matrix3f R_k = T_k.rotation_matrix();
    const Eigen::Vector3f t_k_pos = T_k.translation();

    R_rel[k] = R_bl_T * R_ref_T * R_k * R_bl;
    t_rel[k] = R_bl_T * R_ref_T * (R_k * t_bl + t_k_pos - t_ref) - R_bl_T * t_bl;
  }

  // --- 3. Apply LUT to each point ---
  PointCloud result;
  result.reserve(cloud.size());

  for (const auto& pt : cloud) {
    // Map offset_time to LUT index
    const float t_from_start = pt.offset_time - min_offset;
    int idx = static_cast<int>(t_from_start / static_cast<float>(dt_seg));
    idx = std::max(0, std::min(idx, N - 1));

    // Apply pre-computed relative transform
    const Eigen::Vector3f p_lidar(pt.x, pt.y, pt.z);
    const Eigen::Vector3f p_corrected = R_rel[idx] * p_lidar + t_rel[idx];

    Point3D out_pt;
    out_pt.x           = p_corrected.x();
    out_pt.y           = p_corrected.y();
    out_pt.z           = p_corrected.z();
    out_pt.intensity   = pt.intensity;
    out_pt.offset_time = 0.0f;

    result.push_back(out_pt);
  }

  return result;
}

}  // namespace core
}  // namespace tof_slam
