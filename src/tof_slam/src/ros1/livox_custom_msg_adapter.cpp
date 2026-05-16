// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// livox_custom_msg_adapter.cpp — Livox CustomMsg → core::PointCloud.
//
// Pure format conversion — matches the behavior of PointCloudAdapter:
//   - NaN/Inf guard only (no range, no tag, no duplicate filtering)
//   - offset_time (uint32 nanoseconds) → float seconds
//   - reflectivity (uint8) → float intensity
//
// All geometric filtering (range, stride, voxel) is handled downstream
// by the frontend preprocessing pipeline, identical to PointCloud2 path.

#ifdef HAS_LIVOX_ROS_DRIVER2

#include "tof_slam/ros1/livox_custom_msg_adapter.hpp"

#include <cmath>
#include <spdlog/spdlog.h>

namespace tof_slam {
namespace ros_adapter {

core::PointCloud LivoxCustomMsgAdapter::convert(
    const livox_ros_driver2::CustomMsg::ConstPtr& msg) const {
  if (!msg) {
    return core::PointCloud{};
  }

  const uint32_t n_points = msg->point_num;
  core::PointCloud cloud;
  cloud.reserve(static_cast<size_t>(n_points));

  for (uint32_t i = 0; i < n_points; ++i) {
    const auto& pt = msg->points[i];

    // ---- NaN/Inf guard (same as PointCloudAdapter) ----
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }

    // ---- Build core::Point3D ----
    core::Point3D out;
    out.x = pt.x;
    out.y = pt.y;
    out.z = pt.z;
    out.intensity = static_cast<float>(pt.reflectivity);

    // Time: offset_time is uint32 nanoseconds from scan start.
    // Convert to float seconds — same conversion as PointCloudAdapter's
    // "offset_time" (UINT32 ns) path:
    //   pt.offset_time = static_cast<float>(static_cast<double>(ot_ns) * 1e-9);
    out.offset_time = static_cast<float>(
        static_cast<double>(pt.offset_time) * 1e-9);

    cloud.push_back(out);
  }

  // Debug: log diagnostics for first 3 scans
  {
    static int debug_count = 0;
    if (debug_count < 3 && !cloud.empty()) {
      float min_ot = cloud[0].offset_time;
      float max_ot = cloud[0].offset_time;
      int zero_count = 0;
      for (size_t i = 0; i < cloud.size(); ++i) {
        min_ot = std::min(min_ot, cloud[i].offset_time);
        max_ot = std::max(max_ot, cloud[i].offset_time);
        if (std::abs(cloud[i].offset_time) < 1e-9f) ++zero_count;
      }
      SPDLOG_INFO("[LivoxCustomMsgAdapter] scan={} pts={} "
                  "offset_time: min={:.6f} max={:.6f} zero={}/{}",
                  debug_count, cloud.size(),
                  min_ot, max_ot,
                  zero_count, cloud.size());
      ++debug_count;
    }
  }

  return cloud;
}

}  // namespace ros_adapter
}  // namespace tof_slam

#endif  // HAS_LIVOX_ROS_DRIVER2
