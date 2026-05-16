// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// map_saver.hpp -- Save loop-closure-optimized map for localization mode.
//
// Saves:
//   1. global_map.pcd   — PGO-corrected merged point cloud (PCL PCD binary)
//   2. pose_graph.bin   — Serialized pose graph (nodes + edges)
//   3. submaps/*.pcd    — Individual submap clouds (world frame)
//   4. metadata.yaml    — Config, extrinsic, bounds, statistics

#ifndef TOF_SLAM_BACKEND_MAP_SAVER_HPP_
#define TOF_SLAM_BACKEND_MAP_SAVER_HPP_

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tof_slam/mapping/occupancy_grid.hpp"

namespace tof_slam {

// Forward declarations
class LoopClosureManager;

struct MapSaveConfig {
  double save_voxel_size = 0.05;        // Voxel downsample size for global map (m)
  bool save_submaps = true;              // Also save individual submap PCDs
  bool save_pose_graph = true;           // Save pose graph binary
  std::string map_frame = "map";
  std::string base_frame = "base_link";
  std::string lidar_frame = "front_spot_pcl";
  // Extrinsic (for metadata)
  double extrinsic_x = 0.0;
  double extrinsic_y = 0.0;
  double extrinsic_z = 0.0;
  double extrinsic_roll = 0.0;
  double extrinsic_pitch = 0.0;
  double extrinsic_yaw = 0.0;
  // SurfelMap config (for metadata, used when loading for localization)
  double surfel_l0_voxel_size = 0.2;
  int surfel_l1_hierarchy_factor = 3;
  double surfel_planarity_threshold = 0.13;
  int surfel_min_l0_for_surfel = 2;
};

struct MapSaveResult {
  bool success = false;
  std::string message;
  std::string save_path;
  int num_keyframes = 0;
  int num_submaps = 0;
  int num_loop_closures = 0;
  int total_raw_points = 0;
  int global_map_points = 0;
  Eigen::Vector3d bounds_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d bounds_max = Eigen::Vector3d::Zero();
};

class MapSaver {
 public:
  explicit MapSaver(const MapSaveConfig& config = {});

  /// Save the complete map to the specified directory.
  /// Creates the directory if it doesn't exist.
  /// @param save_dir  Output directory path
  /// @param manager   Loop closure manager with keyframes, pose graph, submaps
  /// @return Result with success status and statistics
  MapSaveResult save(const std::string& save_dir,
                     const LoopClosureManager& manager);

  /// Save occupancy grid map in Nav2/slam_toolbox format (PNG + YAML).
  /// @param save_dir  Output directory (same as global map save dir)
  /// @param grid      GlobalGrid from OccupancyGridGenerator::assembleGlobalGrid()
  /// @return true if saved successfully
  bool saveOccupancyGridMap(const std::string& save_dir,
                            const OccupancyGridGenerator::GlobalGrid& grid);

 private:
  /// Build global point cloud by transforming all keyframe clouds to world frame
  /// using PGO-corrected poses, then voxel downsample.
  MapSaveResult buildAndSaveGlobalMap(const std::string& save_dir,
                                      const LoopClosureManager& manager);

  /// Serialize pose graph to binary file.
  bool savePoseGraph(const std::string& filepath,
                     const LoopClosureManager& manager);

  /// Save individual submap clouds in world frame.
  bool saveSubmaps(const std::string& submap_dir,
                   const LoopClosureManager& manager);

  /// Write metadata YAML.
  bool saveMetadata(const std::string& filepath,
                    const MapSaveResult& result);

  MapSaveConfig config_;
};

}  // namespace tof_slam

#endif  // TOF_SLAM_BACKEND_MAP_SAVER_HPP_
