// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// motion_undistort.hpp — Per-point pose interpolation for motion undistortion.
//
// Corrects LiDAR scan distortion caused by robot motion during scan acquisition.
// Each point is transformed from its capture-time pose to the end-of-scan frame.

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_MOTION_UNDISTORT_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_MOTION_UNDISTORT_HPP_

#include <utility>
#include <vector>

#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/common/types/point_types.hpp"

namespace tof_slam {
namespace core {

/// A timestamped pose for trajectory interpolation.
struct StampedPose {
  double timestamp = 0.0;
  Se3 pose;  // World frame pose at this timestamp
};

/// Interpolate between two SE(3) poses using SLERP on rotation and LERP on
/// translation.
///
/// @param p0     Start pose (alpha = 0.0)
/// @param p1     End pose   (alpha = 1.0)
/// @param alpha  Interpolation factor in [0, 1]
/// @return Interpolated SE(3) pose
Se3 interpolate_pose(const Se3& p0, const Se3& p1, float alpha);

/// Undistort a point cloud by compensating for robot motion during the scan.
///
/// Each point is transformed from its capture-time pose to the end-of-scan
/// pose.  Points with timestamps that fall outside the trajectory window are
/// left unchanged (coordinate preserved, offset_time reset to 0).
///
/// Transform chain per point:
///   p_lidar_end = T_body_lidar^{-1} * T_end^{-1} * T_point * T_body_lidar
///               * p_lidar_capture
///
/// @param cloud          Raw point cloud with offset_time per point (seconds
///                       relative to scan_end_time; typically negative)
/// @param trajectory     Sorted (ascending timestamp) list of world-frame poses
///                       from IMU propagation — must have >= 2 entries to
///                       enable interpolation
/// @param scan_end_time  Absolute timestamp (seconds) of scan end, used as the
///                       reference frame into which all points are projected
/// @param T_body_lidar   Extrinsic: transforms points from LiDAR frame to body
///                       frame (T_body_lidar * p_lidar = p_body)
/// @return Undistorted point cloud (same point count; offset_time reset to 0)
PointCloud undistort_scan(const PointCloud& cloud,
                          const std::vector<StampedPose>& trajectory,
                          double scan_end_time,
                          const Se3& T_body_lidar);

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_MOTION_UNDISTORT_HPP_
