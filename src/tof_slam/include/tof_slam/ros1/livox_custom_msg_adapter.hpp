// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// livox_custom_msg_adapter.hpp — Converts Livox CustomMsg directly to
// core::PointCloud without intermediate PointCloud2 conversion.
//
// Pure format conversion only (NaN filter + type cast).
// Matches PointCloudAdapter behavior so that downstream frontend
// preprocessing produces identical results regardless of input format.

#ifndef TOF_SLAM_ROS1_LIVOX_CUSTOM_MSG_ADAPTER_HPP_
#define TOF_SLAM_ROS1_LIVOX_CUSTOM_MSG_ADAPTER_HPP_

#include "tof_slam/common/types/point_types.hpp"

#ifdef HAS_LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/CustomMsg.h>
#endif

namespace tof_slam {
namespace ros_adapter {

/// Converts Livox CustomMsg to core::PointCloud.
///
/// Performs only format conversion (no geometric filtering):
///   - NaN/Inf guard
///   - offset_time (uint32 ns) → float seconds
///   - reflectivity (uint8) → float intensity
///
/// Range, stride, voxel filtering is handled by the frontend pipeline.
class LivoxCustomMsgAdapter {
 public:
  LivoxCustomMsgAdapter() = default;

#ifdef HAS_LIVOX_ROS_DRIVER2
  /// Convert a Livox CustomMsg to core PointCloud.
  core::PointCloud convert(
      const livox_ros_driver2::CustomMsg::ConstPtr& msg) const;
#endif
};

}  // namespace ros_adapter
}  // namespace tof_slam

#endif  // TOF_SLAM_ROS1_LIVOX_CUSTOM_MSG_ADAPTER_HPP_
