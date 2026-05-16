#pragma once

#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tof_slam/common/types.hpp"
#include "tof_slam/common/config.hpp"
#include "tof_slam/mapping/occupancy_grid.hpp"

namespace tof_slam {

/// A keyframe stores pose, cloud, and covariance at a selected moment
struct Keyframe {
  size_t id{0};
  PoseState state;                                    // pose at keyframe time
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;          // downsampled cloud (body frame)
  Eigen::Matrix<double, 6, 6> registration_cov;      // posterior covariance from registration
  LocalGrid local_grid;                               // OGM asset tied to this keyframe
  bool has_local_grid{false};

  // Relative measurements to previous keyframe
  Eigen::Matrix4d T_from_prev{Eigen::Matrix4d::Identity()};  // relative transform from prev KF
  Eigen::Matrix4d wheel_T_from_prev{Eigen::Matrix4d::Identity()};  // wheel-relative transform from prev KF
  Eigen::Matrix<double, 6, 6> wheel_cov_from_prev{Eigen::Matrix<double, 6, 6>::Identity()};
  Eigen::Matrix<double, 6, 6> scan_cov_from_prev{Eigen::Matrix<double, 6, 6>::Identity()};
};

/// Manages keyframe selection and storage.
class KeyframeManager {
 public:
  explicit KeyframeManager(const TofSlamConfig& config);

  /// Check if a new keyframe should be created given current state
  bool shouldCreateKeyframe(const PoseState& current_state) const;

  /// Add a new keyframe. Returns the keyframe ID.
  size_t addKeyframe(const PoseState& state,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     const Eigen::Matrix<double, 6, 6>& reg_cov,
                     const Eigen::Matrix<double, 6, 6>& wheel_cov,
                     const Eigen::Matrix4d& wheel_T_from_prev = Eigen::Matrix4d::Identity());

  /// Get keyframe by ID
  const Keyframe& getKeyframe(size_t id) const;

  /// Get the latest keyframe
  const Keyframe& getLatestKeyframe() const;

  /// Get all keyframes
  const std::vector<Keyframe>& getAllKeyframes() const { return keyframes_; }

  /// Number of keyframes
  size_t size() const { return keyframes_.size(); }

  /// Update a keyframe's pose (called after backend optimization)
  void updateKeyframePose(size_t id, const PoseState& new_state);

  /// Attach a local occupancy grid asset to an existing keyframe
  void setLocalGrid(size_t id, LocalGrid local_grid);

  /// Check whether a keyframe has an associated local grid
  bool hasLocalGrid(size_t id) const;

  /// Collect all stored local grids in keyframe order
  std::vector<LocalGrid> getLocalGrids() const;

  bool hasKeyframes() const { return !keyframes_.empty(); }

 private:
  const TofSlamConfig& config_;
  std::vector<Keyframe> keyframes_;
};

}  // namespace tof_slam
