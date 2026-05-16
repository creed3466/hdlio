#include "tof_slam/backend/loop_detector.hpp"
#include "tof_slam/common/se3.hpp"

#include <cmath>
#include <algorithm>
#include <pcl/registration/icp.h>

namespace tof_slam {

LoopDetector::LoopDetector(const TofSlamConfig& config)
    : config_(config) {
  max_range_ = config_.map_radius;
  loop_score_threshold_ = config_.loop_score_threshold;
  ring_key_threshold_ = config_.loop_ring_key_threshold;
  icp_fitness_threshold_ = config_.loop_icp_fitness_threshold;
}

ScanContext LoopDetector::computeDescriptor(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    size_t keyframe_id) const {
  ScanContext sc;
  sc.keyframe_id = keyframe_id;

  const double sector_size = 2.0 * M_PI / ScanContext::kNumSectors;
  const double ring_size = max_range_ / ScanContext::kNumRings;

  for (const auto& pt : cloud->points) {
    double x = static_cast<double>(pt.x);
    double y = static_cast<double>(pt.y);
    double range = std::sqrt(x * x + y * y);

    if (range < 1e-6 || range > max_range_) {
      continue;
    }

    // Angle in [0, 2*PI)
    double angle = std::atan2(y, x) + M_PI;

    int angle_idx = static_cast<int>(std::floor(angle / sector_size));
    angle_idx = std::clamp(angle_idx, 0, ScanContext::kNumSectors - 1);

    int range_idx = static_cast<int>(std::floor(range / ring_size));
    range_idx = std::clamp(range_idx, 0, ScanContext::kNumRings - 1);

    // Store max range in each bin
    sc.descriptor(range_idx, angle_idx) =
        std::max(sc.descriptor(range_idx, angle_idx), range);
  }

  // Ring key: row-wise mean of descriptor (for each ring, mean across sectors)
  for (int r = 0; r < ScanContext::kNumRings; ++r) {
    sc.ring_key(r) = sc.descriptor.row(r).mean();
  }

  return sc;
}

void LoopDetector::addDescriptor(const ScanContext& sc) {
  database_.push_back(sc);
}

std::vector<LoopCandidate> LoopDetector::detectLoop(
    const ScanContext& query,
  const std::vector<PoseState>& keyframe_poses,
    int min_keyframe_gap) const {
  std::vector<LoopCandidate> candidates;

  for (const auto& db_sc : database_) {
    // Skip temporally close keyframes
    if (query.keyframe_id <= db_sc.keyframe_id) {
      continue;
    }
    int gap = static_cast<int>(query.keyframe_id - db_sc.keyframe_id);
    if (gap < min_keyframe_gap) {
      continue;
    }

    // Pre-filter with ring key distance
    double rk_dist = ringKeyDistance(query, db_sc);
    if (rk_dist > ring_key_threshold_) {
      continue;
    }

    // Full descriptor distance
    double dist = computeDistance(query, db_sc);
    if (dist < loop_score_threshold_) {
      LoopCandidate cand;
      cand.query_id = query.keyframe_id;
      cand.match_id = db_sc.keyframe_id;
      cand.descriptor_distance = dist;
      if (query.keyframe_id < keyframe_poses.size() &&
          db_sc.keyframe_id < keyframe_poses.size()) {
        const Eigen::Matrix4d T_query =
            se3::toTransform(keyframe_poses[query.keyframe_id].q_wb,
                             keyframe_poses[query.keyframe_id].p_wb);
        const Eigen::Matrix4d T_match =
            se3::toTransform(keyframe_poses[db_sc.keyframe_id].q_wb,
                             keyframe_poses[db_sc.keyframe_id].p_wb);
        cand.T_relative = se3::inverseSE3(T_match) * T_query;
      } else {
        cand.T_relative = Eigen::Matrix4d::Identity();
      }
      cand.information = Eigen::Matrix<double, 6, 6>::Identity();
      cand.verified = false;
      candidates.push_back(cand);
    }
  }

  // Sort by distance (ascending)
  std::sort(candidates.begin(), candidates.end(),
            [](const LoopCandidate& a, const LoopCandidate& b) {
              return a.descriptor_distance < b.descriptor_distance;
            });

  // Return top 3 candidates at most
  if (candidates.size() > 3) {
    candidates.resize(3);
  }

  return candidates;
}

double LoopDetector::computeDistance(const ScanContext& a,
                                     const ScanContext& b) const {
  double min_dist = std::numeric_limits<double>::max();
  const int num_sectors = ScanContext::kNumSectors;

  for (int shift = 0; shift < num_sectors; ++shift) {
    double total_dist = 0.0;
    int valid_cols = 0;

    for (int col = 0; col < num_sectors; ++col) {
      int shifted_col = (col + shift) % num_sectors;

      // Get columns as vectors
      Eigen::VectorXd col_a = a.descriptor.col(col);
      Eigen::VectorXd col_b = b.descriptor.col(shifted_col);

      // Skip if both columns are zero
      if (col_a.norm() < 1e-10 && col_b.norm() < 1e-10) {
        continue;
      }

      total_dist += cosineDistance(col_a, col_b);
      ++valid_cols;
    }

    double avg_dist = (valid_cols > 0) ? (total_dist / valid_cols) : 1.0;
    min_dist = std::min(min_dist, avg_dist);
  }

  return min_dist;
}

double LoopDetector::cosineDistance(const Eigen::VectorXd& a,
                                    const Eigen::VectorXd& b) const {
  double denom = a.norm() * b.norm() + 1e-10;
  return 1.0 - a.dot(b) / denom;
}

double LoopDetector::ringKeyDistance(const ScanContext& a,
                                     const ScanContext& b) const {
  return cosineDistance(a.ring_key, b.ring_key);
}

bool LoopDetector::verifyLoop(
    LoopCandidate& candidate,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& query_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& match_cloud,
    const PoseState& query_pose,
    const PoseState& match_pose) const {
  if (!query_cloud || !match_cloud || query_cloud->empty() || match_cloud->empty()) {
    return false;
  }

  // Compute initial guess: T_init = T_match^{-1} * T_query
  Eigen::Matrix4d T_query = se3::toTransform(query_pose.q_wb, query_pose.p_wb);
  Eigen::Matrix4d T_match = se3::toTransform(match_pose.q_wb, match_pose.p_wb);
  Eigen::Matrix4d T_init = se3::inverseSE3(T_match) * T_query;

  // ICP alignment: align query_cloud to match_cloud
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(query_cloud);
  icp.setInputTarget(match_cloud);
  icp.setMaximumIterations(config_.loop_icp_max_iterations);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setMaxCorrespondenceDistance(config_.loop_icp_max_correspondence_distance);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned, T_init.cast<float>());

  if (icp.hasConverged()) {
    double fitness = icp.getFitnessScore();
    if (fitness < icp_fitness_threshold_) {
      const Eigen::Matrix4d T_icp = icp.getFinalTransformation().cast<double>();
      const Eigen::Matrix4d T_delta = se3::inverseSE3(T_init) * T_icp;
      const Eigen::Matrix<double, 6, 1> xi_delta = se3::LogSE3(T_delta);
      const double rot_delta = xi_delta.head<3>().norm();
      const double trans_delta = xi_delta.tail<3>().norm();

      const double max_rot_delta = std::max(3.0 * config_.keyframe_rot_thresh, 0.35);
      const double max_trans_delta = std::max(3.0 * config_.keyframe_trans_thresh, 0.75);
      if (rot_delta > max_rot_delta || trans_delta > max_trans_delta) {
        return false;
      }

      candidate.T_relative = T_icp;
      // Use inverse of fitness as proxy for information (higher fitness = less certain)
      double info_weight = 1.0 / (fitness + 1e-6);
      info_weight = std::clamp(info_weight, 1.0, 1e3);
      candidate.information = Eigen::Matrix<double, 6, 6>::Identity() * info_weight;
      candidate.verified = true;
      return true;
    }
  }

  return false;
}

}  // namespace tof_slam
