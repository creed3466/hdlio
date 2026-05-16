// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// point_cloud_adapter.hpp — Converts ROS PointCloud2 to core::PointCloud.
//
// Handles multiple Livox / generic LiDAR field layouts:
//   - "timestamp" field (double, absolute seconds) → offset relative to scan_timestamp
//   - "offset_time" field (uint32 nanoseconds)     → convert to float seconds
//   - No time field                                 → offset_time = 0
//   - "intensity" field optional; defaults to 0 when absent

#ifndef TOF_SLAM_ROS1_POINT_CLOUD_ADAPTER_HPP_
#define TOF_SLAM_ROS1_POINT_CLOUD_ADAPTER_HPP_

#include <sensor_msgs/PointCloud2.h>

#include "tof_slam/common/types/point_types.hpp"

namespace tof_slam {
namespace ros_adapter {

/// Converts ROS PointCloud2 messages to core::PointCloud.
///
/// Handles multiple point field layouts:
///   - "timestamp" field (double, absolute seconds) → offset relative to message timestamp
///   - "offset_time" field (uint32 nanoseconds)     → convert to float seconds
///   - No time field                                 → offset_time = 0
///   - "intensity" field detection (present or not)
///
/// NaN/Inf points are silently skipped.
class PointCloudAdapter {
 public:
  PointCloudAdapter() = default;

  /// Convert a PointCloud2 message to core PointCloud.
  /// The scan timestamp is derived from msg->header.stamp.
  ///
  /// @param msg  The ROS PointCloud2 message (non-null).
  /// @return     Converted point cloud with offset_time populated.
  core::PointCloud convert(
      const sensor_msgs::PointCloud2::ConstPtr& msg) const;

  /// Convert with an explicit scan timestamp for offset_time computation.
  /// Useful when the caller wants to override the header stamp (e.g. first-point time).
  ///
  /// @param msg            The ROS PointCloud2 message (non-null).
  /// @param scan_timestamp Reference time in seconds (absolute, same epoch as "timestamp" field).
  /// @return               Converted point cloud with offset_time populated.
  core::PointCloud convert(
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      double scan_timestamp) const;
};

}  // namespace ros_adapter
}  // namespace tof_slam

#endif  // TOF_SLAM_ROS1_POINT_CLOUD_ADAPTER_HPP_
