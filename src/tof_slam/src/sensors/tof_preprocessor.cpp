#include "tof_slam/sensors/tof_preprocessor.hpp"

#include <cmath>
#include <cstring>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

namespace tof_slam {

TofPreprocessor::TofPreprocessor(const TofSlamConfig& config)
    : config_(config) {}

namespace {

double unitToSecondsScale(const std::string& unit) {
  if (unit == "sec" || unit == "s") return 1.0;
  if (unit == "ms") return 1e-3;
  if (unit == "us") return 1e-6;
  if (unit == "ns") return 1e-9;
  // Defensive default: treat unknown unit as seconds (will be caught by parity diagnostics)
  return 1.0;
}

bool isSupportedPointFieldDatatype(uint8_t datatype) {
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::FLOAT32:
    case PointField::FLOAT64:
    case PointField::UINT32:
    case PointField::INT32:
      return true;
    default:
      return false;
  }
}

double readPointFieldAsDouble(const sensor_msgs::msg::PointCloud2& msg,
                              size_t point_index,
                              int field_offset,
                              uint8_t datatype) {
  using sensor_msgs::msg::PointField;
  const size_t base = point_index * msg.point_step + static_cast<size_t>(field_offset);

  switch (datatype) {
    case PointField::FLOAT64: {
      double v = 0.0;
      std::memcpy(&v, msg.data.data() + base, sizeof(double));
      return v;
    }
    case PointField::FLOAT32: {
      float v = 0.0f;
      std::memcpy(&v, msg.data.data() + base, sizeof(float));
      return static_cast<double>(v);
    }
    case PointField::UINT32: {
      uint32_t v = 0;
      std::memcpy(&v, msg.data.data() + base, sizeof(uint32_t));
      return static_cast<double>(v);
    }
    case PointField::INT32: {
      int32_t v = 0;
      std::memcpy(&v, msg.data.data() + base, sizeof(int32_t));
      return static_cast<double>(v);
    }
    default:
      return 0.0;
  }
}

}  // namespace

PreprocessedScan TofPreprocessor::process(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) const {
  // Convert PointCloud2 to PCL
  auto cloud_raw = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud_raw);

  rclcpp::Time stamp(msg->header.stamp);

  // --- Extract per-point timestamps from configurable PointCloud2 field ---
  // Output is per-point relative time from frame start in seconds.
  std::vector<double> raw_timestamps;
  bool has_timestamp_field = false;
  int ts_offset = -1;
  uint8_t ts_datatype = 0;

  for (const auto& field : msg->fields) {
    if (field.name == config_.point_time_field) {
      has_timestamp_field = true;
      ts_offset = static_cast<int>(field.offset);
      ts_datatype = field.datatype;
      break;
    }
  }

  const double frame_time_sec = stamp.seconds();

  if (has_timestamp_field && ts_offset >= 0 && isSupportedPointFieldDatatype(ts_datatype)) {
    const double scale = unitToSecondsScale(config_.point_time_unit);
    const size_t point_step = msg->point_step;
    const size_t num_points = cloud_raw->size();
    raw_timestamps.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
      (void)point_step;
      const double raw = readPointFieldAsDouble(*msg, i, ts_offset, ts_datatype);
      const double t_sec = raw * scale;

      if (config_.point_time_reference == "absolute") {
        raw_timestamps.push_back(t_sec - frame_time_sec);
      } else {
        raw_timestamps.push_back(t_sec);
      }
    }
  }

  // Process cloud (filter, downsample)
  PreprocessedScan result = processCloud(cloud_raw, stamp);

  // Attach per-point timestamps for downsampled cloud
  // Map: raw point index -> timestamp via spatial matching (exact XYZ lookup)
  if (!raw_timestamps.empty() && result.cloud && !result.cloud->empty()) {
    // Build position → timestamp map from raw cloud
    std::unordered_map<uint64_t, double> coord_to_ts;
    coord_to_ts.reserve(cloud_raw->size());
    for (size_t i = 0; i < cloud_raw->size(); ++i) {
      const auto& pt = cloud_raw->points[i];
      // Encode (x, y, z) as a compact key (mm precision)
      const int32_t ix = static_cast<int32_t>(std::round(pt.x * 1000.0f));
      const int32_t iy = static_cast<int32_t>(std::round(pt.y * 1000.0f));
      const int32_t iz = static_cast<int32_t>(std::round(pt.z * 1000.0f));
      const uint64_t key =
          (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 40) |
          (static_cast<uint64_t>(static_cast<uint32_t>(iy)) << 20) |
          (static_cast<uint64_t>(static_cast<uint32_t>(iz) & 0xFFFFF));
      coord_to_ts.emplace(key, raw_timestamps[i]);
    }

    result.point_timestamps.reserve(result.cloud->size());
    size_t matched = 0;
    for (const auto& pt : result.cloud->points) {
      const int32_t ix = static_cast<int32_t>(std::round(pt.x * 1000.0f));
      const int32_t iy = static_cast<int32_t>(std::round(pt.y * 1000.0f));
      const int32_t iz = static_cast<int32_t>(std::round(pt.z * 1000.0f));
      const uint64_t key =
          (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 40) |
          (static_cast<uint64_t>(static_cast<uint32_t>(iy)) << 20) |
          (static_cast<uint64_t>(static_cast<uint32_t>(iz) & 0xFFFFF));
      auto it = coord_to_ts.find(key);
      if (it != coord_to_ts.end()) {
        result.point_timestamps.push_back(it->second);
        ++matched;
      } else {
        result.point_timestamps.push_back(0.0);
      }
    }

    // If most timestamps failed to map, drop them to avoid false "has timestamps" signals.
    const double ratio = static_cast<double>(matched) / static_cast<double>(result.cloud->size());
    if (ratio < 0.5) {
      result.point_timestamps.clear();
    }
  }

  return result;
}

PreprocessedScan TofPreprocessor::processCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    const rclcpp::Time& stamp) const {
  PreprocessedScan result;
  result.stamp = stamp;
  result.raw_count = cloud_in->size();

  // --- Step 1: Range filter (remove invalid, out-of-range, NaN points) ---
  auto cloud_valid = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud_valid->reserve(cloud_in->size());

  const double range_min_sq = config_.tof_range_min * config_.tof_range_min;
  const double range_max_sq = config_.tof_range_max * config_.tof_range_max;

  for (const auto& pt : *cloud_in) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    const double r_sq = static_cast<double>(pt.x) * pt.x +
                        static_cast<double>(pt.y) * pt.y +
                        static_cast<double>(pt.z) * pt.z;
    if (r_sq >= range_min_sq && r_sq <= range_max_sq) {
      cloud_valid->push_back(pt);
    }
  }

  if (cloud_valid->empty()) {
    result.cloud = cloud_valid;
    result.weights.clear();
    result.filtered_count = 0;
    return result;
  }

  // --- Step 2: Optional coarse presample for dense clouds ---
  auto cloud_for_sor = adaptivePresample(cloud_valid);

  // --- Step 3: Statistical outlier removal ---
  auto cloud_filtered = removeOutliers(cloud_for_sor);

  if (cloud_filtered->empty()) {
    result.cloud = cloud_filtered;
    result.weights.clear();
    result.filtered_count = 0;
    return result;
  }

  // --- Step 4: Voxel grid downsampling (skip if tof_voxel_size <= 0) ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down;
  if (config_.tof_voxel_size > 0.0) {
    cloud_down = voxelDownsample(cloud_filtered);
  } else {
    cloud_down = cloud_filtered;
  }

  // --- Step 5: Compute range-based weights ---
  auto weights = computeWeights(cloud_down);

  result.cloud = cloud_down;
  result.weights = std::move(weights);
  result.filtered_count = cloud_down->size();
  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TofPreprocessor::adaptivePresample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
  if (!cloud) {
    return cloud;
  }

  if (config_.preprocess_sor_point_threshold <= 0 ||
      config_.preprocess_presample_voxel_size <= 0.0 ||
      cloud->size() < static_cast<size_t>(config_.preprocess_sor_point_threshold)) {
    return cloud;
  }

  return voxelDownsample(cloud, config_.preprocess_presample_voxel_size);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TofPreprocessor::removeOutliers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
  if (cloud->size() < static_cast<size_t>(config_.statistical_outlier_mean_k + 1)) {
    // Not enough points for SOR, return as-is
    return cloud;
  }

  auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(static_cast<int>(config_.statistical_outlier_mean_k));
  sor.setStddevMulThresh(config_.statistical_outlier_stddev);
  sor.filter(*filtered);

  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TofPreprocessor::voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double leaf_size) const {
  auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);

  const float leaf = static_cast<float>(leaf_size > 0.0 ? leaf_size : config_.tof_voxel_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*filtered);

  return filtered;
}

std::vector<double> TofPreprocessor::computeWeights(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
  const double range_max_sq = config_.tof_range_max * config_.tof_range_max;

  std::vector<double> weights;
  weights.reserve(cloud->size());

  for (const auto& pt : *cloud) {
    const double r_sq = static_cast<double>(pt.x) * pt.x +
                        static_cast<double>(pt.y) * pt.y +
                        static_cast<double>(pt.z) * pt.z;
    const double w = std::max(1.0 - r_sq / range_max_sq, 0.0);
    weights.push_back(w);
  }

  return weights;
}

}  // namespace tof_slam
