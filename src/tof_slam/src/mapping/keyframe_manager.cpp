#include "tof_slam/mapping/keyframe_manager.hpp"

#include <cassert>

#include "tof_slam/common/se3.hpp"

namespace tof_slam {

KeyframeManager::KeyframeManager(const TofSlamConfig& config)
    : config_(config) {}

bool KeyframeManager::shouldCreateKeyframe(const PoseState& current_state) const {
  // First keyframe is always created
  if (keyframes_.empty()) {
    return true;
  }

  const auto& latest = keyframes_.back();

  // Compute relative transform: T_rel = T_latest^{-1} * T_current
  const Eigen::Matrix4d T_latest =
      se3::toTransform(latest.state.q_wb, latest.state.p_wb);
  const Eigen::Matrix4d T_current =
      se3::toTransform(current_state.q_wb, current_state.p_wb);
  const Eigen::Matrix4d T_rel = se3::inverseSE3(T_latest) * T_current;

  // Translation delta
  const double trans_delta = T_rel.block<3, 1>(0, 3).norm();

  // Rotation delta via LogSO3
  const Eigen::Vector3d phi = se3::LogSO3(T_rel.block<3, 3>(0, 0));
  const double rot_delta = phi.norm();

  // ||Δp|| > τ_p  OR  ||Δθ|| > τ_r
  return (trans_delta > config_.keyframe_trans_thresh) ||
         (rot_delta > config_.keyframe_rot_thresh);
}

size_t KeyframeManager::addKeyframe(
    const PoseState& state,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Matrix<double, 6, 6>& reg_cov,
  const Eigen::Matrix<double, 6, 6>& wheel_cov,
  const Eigen::Matrix4d& wheel_T_from_prev) {
  Keyframe kf;
  kf.id = keyframes_.size();
  kf.state = state;
  kf.cloud = cloud;
  kf.registration_cov = reg_cov;

  if (!keyframes_.empty()) {
    const auto& prev = keyframes_.back();

    // Relative transform from previous keyframe: T_from_prev = T_prev^{-1} * T_current
    const Eigen::Matrix4d T_prev =
        se3::toTransform(prev.state.q_wb, prev.state.p_wb);
    const Eigen::Matrix4d T_current =
        se3::toTransform(state.q_wb, state.p_wb);
    kf.T_from_prev = se3::inverseSE3(T_prev) * T_current;

    // Store edge covariances for the pose graph
    kf.wheel_T_from_prev = wheel_T_from_prev;
    kf.wheel_cov_from_prev = wheel_cov;
    kf.scan_cov_from_prev = reg_cov;
  }

  const size_t new_id = kf.id;
  keyframes_.push_back(std::move(kf));
  return new_id;
}

const Keyframe& KeyframeManager::getKeyframe(size_t id) const {
  assert(id < keyframes_.size() && "Keyframe ID out of bounds");
  return keyframes_[id];
}

const Keyframe& KeyframeManager::getLatestKeyframe() const {
  assert(!keyframes_.empty() && "No keyframes available");
  return keyframes_.back();
}

void KeyframeManager::updateKeyframePose(size_t id, const PoseState& new_state) {
  assert(id < keyframes_.size() && "Keyframe ID out of bounds");
  keyframes_[id].state = new_state;
}

void KeyframeManager::setLocalGrid(size_t id, LocalGrid local_grid) {
  assert(id < keyframes_.size() && "Keyframe ID out of bounds");
  local_grid.keyframe_id = id;
  keyframes_[id].local_grid = std::move(local_grid);
  keyframes_[id].has_local_grid = true;
}

bool KeyframeManager::hasLocalGrid(size_t id) const {
  assert(id < keyframes_.size() && "Keyframe ID out of bounds");
  return keyframes_[id].has_local_grid;
}

std::vector<LocalGrid> KeyframeManager::getLocalGrids() const {
  std::vector<LocalGrid> local_grids;
  local_grids.reserve(keyframes_.size());
  for (const auto& keyframe : keyframes_) {
    if (keyframe.has_local_grid) {
      local_grids.push_back(keyframe.local_grid);
    }
  }
  return local_grids;
}

}  // namespace tof_slam
