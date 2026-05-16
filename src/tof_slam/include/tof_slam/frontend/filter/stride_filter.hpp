// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// stride_filter.hpp — Keep every N-th point (stride downsampling).

#ifndef TOF_SLAM_FRONTEND_FILTER_STRIDE_FILTER_HPP_
#define TOF_SLAM_FRONTEND_FILTER_STRIDE_FILTER_HPP_

#include "tof_slam/common/types/point_types.hpp"

namespace tof_slam {
namespace core {

/// Keep every `stride`-th point.  stride=1 is identity (no skipping).
/// If stride <= 0, returns an empty cloud.
inline PointCloud stride_filter(const PointCloud& input, int stride) {
  PointCloud output;
  if (stride <= 0 || input.empty()) return output;

  const size_t n = input.size();
  output.reserve((n + static_cast<size_t>(stride) - 1) /
                 static_cast<size_t>(stride));
  for (size_t i = 0; i < n; i += static_cast<size_t>(stride)) {
    output.push_back(input[i]);
  }
  return output;
}

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_FILTER_STRIDE_FILTER_HPP_
