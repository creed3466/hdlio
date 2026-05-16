#pragma once

#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tof_slam/common/types.hpp"
#include "tof_slam/common/config.hpp"

namespace tof_slam {

/// Scan Context descriptor for place recognition.
struct ScanContext {
  static constexpr int kNumSectors = 60;    // angular bins
  static constexpr int kNumRings = 20;      // radial bins

  Eigen::MatrixXd descriptor;              // kNumRings x kNumSectors
  Eigen::VectorXd ring_key;                // row-wise mean for fast retrieval
  size_t keyframe_id{0};

  ScanContext() : descriptor(Eigen::MatrixXd::Zero(kNumRings, kNumSectors)),
                  ring_key(Eigen::VectorXd::Zero(kNumRings)) {}
};

/// Loop closure candidate
struct LoopCandidate {
  size_t query_id;
  size_t match_id;
  double descriptor_distance;
  Eigen::Matrix4d T_relative;      // relative transform from geometric verification
  Eigen::Matrix<double, 6, 6> information;  // information matrix
  bool verified{false};
};

/// Loop closure detector using Scan Context descriptors
class LoopDetector {
 public:
  explicit LoopDetector(const TofSlamConfig& config);

  /// Compute Scan Context descriptor from a point cloud (body frame)
  ScanContext computeDescriptor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                size_t keyframe_id) const;

  /// Add a descriptor to the database
  void addDescriptor(const ScanContext& sc);

  /// Detect loop closure candidates for a query descriptor
  /// Only searches keyframes with sufficient temporal/spatial gap
  std::vector<LoopCandidate> detectLoop(
      const ScanContext& query,
      const std::vector<PoseState>& keyframe_poses,
      int min_keyframe_gap = 20) const;

  /// Geometric verification using point-to-point ICP
  bool verifyLoop(LoopCandidate& candidate,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& query_cloud,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& match_cloud,
                  const PoseState& query_pose,
                  const PoseState& match_pose) const;

  /// Get number of descriptors in database
  size_t size() const { return database_.size(); }

 private:
  /// Compute distance between two scan contexts (min over column shifts)
  double computeDistance(const ScanContext& a, const ScanContext& b) const;

  /// Cosine distance between two vectors
  double cosineDistance(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const;

  /// Ring key distance for fast pre-filtering
  double ringKeyDistance(const ScanContext& a, const ScanContext& b) const;

  const TofSlamConfig& config_;
  std::vector<ScanContext> database_;

  double max_range_{20.0};          // max range for descriptor
  double loop_score_threshold_{0.3}; // descriptor distance threshold
  double ring_key_threshold_{0.5};   // ring key pre-filter threshold
  double icp_fitness_threshold_{0.3}; // ICP fitness threshold for verification
};

}  // namespace tof_slam
