// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// range_filter.hpp — Distance-based point cloud filtering.

#ifndef TOF_SLAM_FRONTEND_FILTER_RANGE_FILTER_HPP_
#define TOF_SLAM_FRONTEND_FILTER_RANGE_FILTER_HPP_

#include "tof_slam/common/types/point_types.hpp"

namespace tof_slam {
namespace core {

/// Keep only points whose distance from origin is in [min_range, max_range].
inline PointCloud range_filter(const PointCloud& input,
                               float min_range,
                               float max_range) {
  PointCloud output;
  if (input.empty() || min_range > max_range) return output;

  const float min_sq = min_range * min_range;
  const float max_sq = max_range * max_range;

  output.reserve(input.size());
  for (const auto& p : input) {
    const float r_sq = p.squared_norm();
    if (r_sq >= min_sq && r_sq <= max_sq) {
      output.push_back(p);
    }
  }
  return output;
}

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_FILTER_RANGE_FILTER_HPP_
