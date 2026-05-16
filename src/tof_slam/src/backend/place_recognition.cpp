#include "tof_slam/backend/place_recognition.hpp"

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace tof_slam {

PlaceRecognition::PlaceRecognition(const TofSlamConfig& config)
    : config_(config) {
  orb_detector_ = cv::ORB::create(max_features_);
}

bool PlaceRecognition::isValidImage(const cv::Mat& image) {
  return !image.empty() && image.rows > 0 && image.cols > 0;
}

void PlaceRecognition::addKeyframe(size_t kf_id, const cv::Mat& image) {
  if (!isValidImage(image)) return;

  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  orb_detector_->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);

  if (descriptors.empty()) return;

  descriptors_db_[kf_id] = descriptors.clone();
  keyframe_ids_.push_back(kf_id);
}

std::vector<PlaceCandidate> PlaceRecognition::query(
    const cv::Mat& image, int top_k) const {
  std::vector<PlaceCandidate> candidates;

  if (!isValidImage(image) || descriptors_db_.empty()) {
    return candidates;
  }

  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat query_desc;
  orb_detector_->detectAndCompute(gray, cv::noArray(), keypoints, query_desc);

  if (query_desc.empty()) return candidates;

  // Compare against all database entries
  for (const auto& [kf_id, db_desc] : descriptors_db_) {
    int num_matches = 0;
    double score = computeSimilarity(query_desc, db_desc, num_matches);

    if (num_matches >= min_matches_) {
      PlaceCandidate cand;
      cand.keyframe_id = kf_id;
      cand.score = score;
      cand.num_matches = num_matches;
      candidates.push_back(cand);
    }
  }

  // Sort by score descending
  std::sort(candidates.begin(), candidates.end(),
            [](const PlaceCandidate& a, const PlaceCandidate& b) {
              return a.score > b.score;
            });

  // Return top-K
  if (static_cast<int>(candidates.size()) > top_k) {
    candidates.resize(top_k);
  }

  return candidates;
}

double PlaceRecognition::computeSimilarity(
    const cv::Mat& desc_a, const cv::Mat& desc_b, int& num_matches) const {
  num_matches = 0;

  if (desc_a.empty() || desc_b.empty()) return 0.0;

  // BFMatcher with Hamming distance for ORB (binary descriptors)
  cv::BFMatcher matcher(cv::NORM_HAMMING);

  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher.knnMatch(desc_a, desc_b, knn_matches, 2);

  // Lowe's ratio test
  int good_matches = 0;
  for (const auto& m : knn_matches) {
    if (m.size() >= 2) {
      if (m[0].distance < match_ratio_threshold_ * m[1].distance) {
        ++good_matches;
      }
    }
  }

  num_matches = good_matches;

  // Normalize score: ratio of good matches to total features
  int total = std::min(desc_a.rows, desc_b.rows);
  if (total == 0) return 0.0;

  return static_cast<double>(good_matches) / total;
}

}  // namespace tof_slam
