#pragma once

#include <vector>
#include <Eigen/Core>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tof_slam/common/config.hpp"

namespace tof_slam {

/// Preprocessed scan output
struct PreprocessedScan {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // filtered & downsampled points in sensor frame
  std::vector<double> weights;                 // per-point range-based weight
  std::vector<double> point_timestamps;        // per-point relative time from frame start (sec), empty if unavailable
  rclcpp::Time stamp;
  size_t raw_count{0};
  size_t filtered_count{0};
};

/// 3D ToF/LiDAR preprocessing: range filter, statistical outlier removal, voxel downsample
/// [Algorithm.md §5.1]
class TofPreprocessor {
 public:
  explicit TofPreprocessor(const TofSlamConfig& config);

  /// Process a PointCloud2 message into filtered point cloud
  PreprocessedScan process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) const;

  /// Process a raw PCL point cloud directly (for testing / offline use)
  PreprocessedScan processCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                const rclcpp::Time& stamp) const;

 private:
  /// Optional coarse voxel downsample before SOR on dense clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr adaptivePresample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

  /// Statistical outlier removal
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

  /// Voxel grid downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double leaf_size = -1.0) const;

  /// Compute range-based weights for each point
  std::vector<double> computeWeights(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

  const TofSlamConfig& config_;
};

}  // namespace tof_slam
