// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// submap.hpp -- SubMap data structure for loop closure detection.
//
// A SubMap accumulates N consecutive keyframes' point clouds into a single
// dense representation suitable for GICP-based loop closure verification.

#ifndef TOF_SLAM_BACKEND_SUBMAP_HPP_
#define TOF_SLAM_BACKEND_SUBMAP_HPP_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tof_slam/common/types.hpp"

namespace tof_slam {

/// A SubMap aggregates point clouds from multiple keyframes into one
/// dense cloud, referenced to the first keyframe's pose.
struct SubMap {
  size_t id{0};                              // SubMap index
  size_t first_keyframe_id{0};               // First contributing keyframe
  size_t last_keyframe_id{0};                // Last contributing keyframe
  size_t num_keyframes{0};                   // Number of contributing keyframes
  std::vector<size_t> keyframe_ids;          // Source-of-truth contributing keyframe IDs

  PoseState reference_pose;                  // Pose of the reference (first) keyframe
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};  // Mean position of all contributing keyframes

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // Accumulated cloud (in reference_pose frame)
  size_t total_points{0};

  SubMap() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {}
};

}  // namespace tof_slam

#endif  // TOF_SLAM_BACKEND_SUBMAP_HPP_
