#include "tof_slam/mapping/map_aligner.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

namespace tof_slam {

namespace {

/// Normalize angle to [0, pi/2) range (90-degree periodicity for walls).
double normalizeToQuadrant(double theta) {
  // Map to [0, pi)
  while (theta < 0.0) theta += M_PI;
  while (theta >= M_PI) theta -= M_PI;
  // Map [pi/2, pi) to [0, pi/2) — walls are 90° periodic
  if (theta >= M_PI_2) theta -= M_PI_2;
  return theta;
}

/// Normalize alignment angle to (-pi/4, pi/4] to minimize rotation.
double normalizeAlignmentAngle(double theta) {
  // theta is in [0, pi/2)
  if (theta > M_PI_4) {
    theta -= M_PI_2;  // e.g., 60° → -30°
  }
  return theta;
}

}  // namespace

AlignmentResult MapAligner::detectAlignment(
    const OccupancyGridGenerator::GlobalGrid& grid,
    int occupied_thresh,
    int min_line_length,
    int min_lines,
    double min_confidence) {
  AlignmentResult result;

  if (grid.width <= 0 || grid.height <= 0 || grid.data.empty()) {
    spdlog::warn("[MapAligner] Empty grid, skipping alignment.");
    return result;
  }

  // Step 1: Convert GlobalGrid to binary image (occupied=white, else=black)
  cv::Mat binary(grid.height, grid.width, CV_8UC1, cv::Scalar(0));

  int occupied_count = 0;
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      int ogm_idx = y * grid.width + x;
      int img_row = grid.height - 1 - y;  // Y-flip

      int8_t val = grid.data[ogm_idx];
      if (val >= 0 && val >= occupied_thresh) {
        binary.at<uint8_t>(img_row, x) = 255;
        ++occupied_count;
      }
    }
  }

  if (occupied_count < 50) {
    spdlog::warn("[MapAligner] Too few occupied cells ({}), skipping alignment.",
                 occupied_count);
    return result;
  }

  // Step 2: Morphological preprocessing to clean noise
  // Close (dilate+erode) to connect nearby wall segments, then open to remove
  // small noise clusters. This preserves wall structure while suppressing
  // radial ToF noise patterns.
  cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat kernel_open  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
  cv::Mat cleaned;
  cv::morphologyEx(binary, cleaned, cv::MORPH_CLOSE, kernel_close);
  cv::morphologyEx(cleaned, cleaned, cv::MORPH_OPEN, kernel_open);

  // Step 3: Canny edge detection to extract wall boundaries
  cv::Mat edges;
  cv::Canny(cleaned, edges, 50, 150);

  // Step 4: Detect line segments using Probabilistic Hough Transform
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(
      edges,
      lines,
      1.0,               // rho resolution (pixels)
      CV_PI / 360.0,     // theta resolution (0.5 degree)
      15,                // accumulator threshold
      min_line_length,   // minimum line length (pixels)
      5                  // maximum line gap (pixels)
  );

  result.num_lines_detected = static_cast<int>(lines.size());

  spdlog::info("[MapAligner] Hough: {} lines from {} occupied cells",
               result.num_lines_detected, occupied_count);

  if (result.num_lines_detected < min_lines) {
    spdlog::info("[MapAligner] Only {} lines detected (min={}), skipping.",
                 result.num_lines_detected, min_lines);
    return result;
  }

  // Step 5: Build length-weighted angle histogram
  // Bin width = 0.5 degrees, 180 bins for [0, 90°)
  constexpr int kNumBins = 180;
  constexpr double kBinWidth = M_PI_2 / kNumBins;  // 0.5° in radians
  std::vector<double> histogram(kNumBins, 0.0);
  double total_weight = 0.0;

  for (const auto& line : lines) {
    double dx = static_cast<double>(line[2] - line[0]);
    double dy = static_cast<double>(line[3] - line[1]);
    double length = std::sqrt(dx * dx + dy * dy);
    double theta = std::atan2(dy, dx);

    // Weight by length squared to strongly favor long walls over noise
    double weight = length * length;

    theta = normalizeToQuadrant(theta);

    int bin = static_cast<int>(std::floor(theta / kBinWidth));
    bin = std::clamp(bin, 0, kNumBins - 1);
    histogram[bin] += weight;
    total_weight += weight;
  }

  if (total_weight < 1e-6) {
    return result;
  }

  // Step 6: Gaussian smoothing (sigma = 3 bins, 7-tap kernel)
  std::vector<double> smoothed(kNumBins, 0.0);
  const double kernel[] = {
    0.006, 0.061, 0.242, 0.383, 0.242, 0.061, 0.006
  };
  for (int i = 0; i < kNumBins; ++i) {
    for (int k = -3; k <= 3; ++k) {
      int idx = (i + k + kNumBins) % kNumBins;
      smoothed[i] += histogram[idx] * kernel[k + 3];
    }
  }

  // Step 7: Find peak
  int peak_bin = 0;
  double peak_val = 0.0;
  for (int i = 0; i < kNumBins; ++i) {
    if (smoothed[i] > peak_val) {
      peak_val = smoothed[i];
      peak_bin = i;
    }
  }

  // Sub-bin interpolation using parabolic fit on 3 neighbors
  double peak_angle = (peak_bin + 0.5) * kBinWidth;
  if (peak_bin > 0 && peak_bin < kNumBins - 1) {
    double y_left  = smoothed[peak_bin - 1];
    double y_mid   = smoothed[peak_bin];
    double y_right = smoothed[peak_bin + 1];
    double denom = 2.0 * (2.0 * y_mid - y_left - y_right);
    if (std::abs(denom) > 1e-12) {
      double delta = (y_left - y_right) / denom;
      peak_angle = (peak_bin + 0.5 + delta) * kBinWidth;
    }
  }

  // Confidence: sum of bins within ±5° of peak / total weight
  // This measures how concentrated the histogram is around the peak
  int half_window = static_cast<int>(std::ceil(5.0 * M_PI / 180.0 / kBinWidth));
  double peak_region_weight = 0.0;
  for (int k = -half_window; k <= half_window; ++k) {
    int idx = (peak_bin + k + kNumBins) % kNumBins;
    peak_region_weight += smoothed[idx];
  }
  double confidence = peak_region_weight / total_weight;
  result.confidence = confidence;

  spdlog::info("[MapAligner] Peak at {:.1f}°, confidence={:.3f} (±5° region)",
               peak_angle * 180.0 / M_PI, confidence);

  if (confidence < min_confidence) {
    spdlog::info("[MapAligner] Low confidence ({:.3f} < {:.3f}), skipping.",
                 confidence, min_confidence);
    return result;
  }

  // Step 8: Normalize to (-pi/4, pi/4] to minimize rotation amount
  double alignment_angle = normalizeAlignmentAngle(peak_angle);

  // Skip if angle is very small (already well-aligned)
  if (std::abs(alignment_angle) < 0.5 * M_PI / 180.0) {  // < 0.5 degree
    spdlog::info("[MapAligner] Already well-aligned (angle={:.2f}°), skipping.",
                 alignment_angle * 180.0 / M_PI);
    result.confidence = confidence;
    return result;
  }

  result.angle_rad = alignment_angle;
  result.aligned = true;

  spdlog::info("[MapAligner] Detected alignment: angle={:.2f}° ({:.4f} rad), "
               "confidence={:.3f}, lines={}, occupied={}",
               alignment_angle * 180.0 / M_PI, alignment_angle,
               confidence, result.num_lines_detected, occupied_count);

  return result;
}

void MapAligner::rotatePoses(
    std::vector<Eigen::Matrix4d>& poses,
    double angle_rad) {
  for (auto& pose : poses) {
    pose = rotatePose(pose, angle_rad);
  }
}

Eigen::Matrix4d MapAligner::rotatePose(
    const Eigen::Matrix4d& pose,
    double angle_rad) {
  Eigen::AngleAxisd rot_z(-angle_rad, Eigen::Vector3d::UnitZ());
  Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
  R.block<3, 3>(0, 0) = rot_z.toRotationMatrix();
  return R * pose;
}

}  // namespace tof_slam
