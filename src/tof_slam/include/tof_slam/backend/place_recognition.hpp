#pragma once

#include <vector>
#include <unordered_map>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "tof_slam/common/types.hpp"
#include "tof_slam/common/config.hpp"

namespace tof_slam {

/// BoW-like place recognition using ORB features
/// Lightweight implementation without FBoW dependency
/// Uses brute-force descriptor matching with inverted index concept
struct PlaceCandidate {
  size_t keyframe_id;
  double score;           // similarity score [0, 1]
  int num_matches;        // number of feature matches
};

/// Place recognition module for kidnap recovery.
class PlaceRecognition {
 public:
  explicit PlaceRecognition(const TofSlamConfig& config);

  /// Add a keyframe image to the database
  /// Extracts ORB features and stores descriptor for retrieval
  void addKeyframe(size_t kf_id, const cv::Mat& image);

  /// Query the database for similar places
  /// Returns top-K candidates sorted by score (descending)
  std::vector<PlaceCandidate> query(const cv::Mat& image, int top_k = 5) const;

  /// Get number of entries in database
  size_t size() const { return descriptors_db_.size(); }

  /// Check if image input is available (non-empty)
  static bool isValidImage(const cv::Mat& image);

 private:
  /// Compute ORB descriptor similarity using BF matching
  double computeSimilarity(const cv::Mat& desc_a, const cv::Mat& desc_b,
                           int& num_matches) const;

  const TofSlamConfig& config_;
  cv::Ptr<cv::ORB> orb_detector_;

  // Database: keyframe_id -> descriptors
  std::unordered_map<size_t, cv::Mat> descriptors_db_;
  std::vector<size_t> keyframe_ids_;  // ordered list for iteration

  // Matching parameters
  int min_matches_{10};
  double match_ratio_threshold_{0.7};  // Lowe's ratio test
  int max_features_{500};
};

}  // namespace tof_slam
