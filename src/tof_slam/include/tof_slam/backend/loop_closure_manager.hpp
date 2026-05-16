// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// loop_closure_manager.hpp -- Spatial-proximity + GICP loop closure pipeline.
//
// Designed for low-cost ToF LiDAR (~300 pts/scan, ~8m range).
// Strategy:
//   1. Accumulate N keyframes into a SubMap (~9-15K points)
//   2. Spatial proximity detection: current submap vs old submaps
//   3. GICP verification on accumulated submap pairs
//   4. Feed verified loop edges into PoseGraph
//   5. Optimise and produce T_map_odom correction
//
// All operations are designed to run on a background thread.

#ifndef TOF_SLAM_BACKEND_LOOP_CLOSURE_MANAGER_HPP_
#define TOF_SLAM_BACKEND_LOOP_CLOSURE_MANAGER_HPP_

#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tof_slam/backend/submap.hpp"
#include "tof_slam/backend/pose_graph.hpp"
#include "tof_slam/backend/pgo_debug_logger.hpp"
#include "tof_slam/backend/pgo_convergence_monitor.hpp"
#include "tof_slam/mapping/keyframe_manager.hpp"
#include "tof_slam/common/config.hpp"
#include "tof_slam/common/se3.hpp"

namespace tof_slam {

/// GICP match diagnostics for monitoring.
struct GicpMatchResult {
  size_t query_submap_id{0};
  size_t match_submap_id{0};
  size_t query_keyframe_id{0};
  size_t match_keyframe_id{0};

  // GICP results
  double fitness_score{1e6};        // Mean squared distance of inliers
  int num_correspondences{0};       // Retained for backward-compatible debug output
  int initial_inlier_count{0};      // Initial-guess inliers before GICP
  int final_inlier_count{0};        // Final inliers after GICP
  bool converged{false};            // Did GICP converge?
  bool verification_passed{false};  // Passed geometric verification?
  bool accepted{false};             // Passed all verification checks?
  int gicp_iterations{0};           // Actual iterations used
  double match_score{0.0};          // Ranking score among passing candidates

  // Relative transform from GICP
  Eigen::Matrix4d T_relative{Eigen::Matrix4d::Identity()};
  double translation_delta{0.0};    // ||t|| of relative transform (m)
  double rotation_delta{0.0};       // ||phi|| of relative transform (rad)

  // Spatial context
  double spatial_distance{0.0};     // Distance between submap centroids (m)
  int keyframe_gap{0};              // Temporal gap in keyframes

  // Timing
  double gicp_time_ms{0.0};
  double total_time_ms{0.0};

  // Point counts and overlap diagnostics
  size_t query_points{0};
  size_t match_points{0};
  double heading_delta_rad{0.0};
  double initial_source_overlap_ratio{0.0};
  double initial_target_overlap_ratio{0.0};
  double source_overlap_ratio{0.0};
  double target_overlap_ratio{0.0};
  double median_inlier_distance{0.0};
  double p90_inlier_distance{0.0};
  std::string reject_reason{"not_evaluated"};

  // Submap centroid positions (for debug visualization)
  double query_centroid_x = 0.0;
  double query_centroid_y = 0.0;
  double query_centroid_z = 0.0;
  double match_centroid_x = 0.0;
  double match_centroid_y = 0.0;
  double match_centroid_z = 0.0;
};

/// Cumulative loop closure statistics.
struct LoopClosureStats {
  int total_submaps{0};
  int total_candidates_tested{0};
  int total_gicp_converged{0};
  int total_loops_accepted{0};
  int total_loops_rejected{0};
  int total_optimizations{0};
  double last_optimization_time_ms{0.0};
  double last_pgo_cost_before{0.0};
  double last_pgo_cost_after{0.0};
  double last_pgo_mean_node_shift_m{0.0};
  double last_pgo_max_node_shift_m{0.0};
  double last_pgo_latest_node_shift_m{0.0};
  double last_pgo_mean_loop_residual_before{0.0};
  double last_pgo_mean_loop_residual_after{0.0};
  double last_pgo_max_loop_residual_before{0.0};
  double last_pgo_max_loop_residual_after{0.0};
  Eigen::Matrix4d T_map_odom{Eigen::Matrix4d::Identity()};
};

struct PoseGraphSnapshot {
  std::vector<PoseGraphNode> nodes;
  std::vector<PoseGraphEdge> edges;
};

struct KeyframePoseSnapshot {
  size_t id{0};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
};

struct VisualSubmapSnapshot {
  size_t id{0};
  size_t first_keyframe_id{0};
  size_t last_keyframe_id{0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud;

  VisualSubmapSnapshot() : world_cloud(new pcl::PointCloud<pcl::PointXYZ>) {}
};

struct LoopClosureDebugSnapshot {
  PoseGraphSnapshot pose_graph;
  std::vector<KeyframePoseSnapshot> keyframes;
  std::vector<VisualSubmapSnapshot> visual_submaps;
  std::vector<GicpMatchResult> last_results;
};

/// Loop closure configuration.
struct LoopClosureConfig {
  // SubMap accumulation
  int submap_keyframe_count = 30;           // Keyframes per submap
  double submap_voxel_size = 0.1;           // Voxel downsampling for accumulated cloud (m)

  // Loop detection (spatial proximity)
  double loop_search_radius = 5.0;          // Max distance to search for loop candidates (m)
  int loop_min_keyframe_gap = 30;           // Min keyframe gap between query and candidate
  int loop_min_submap_gap = 2;              // Min submap gap
  int max_candidates_per_query = 3;         // Max candidates to test per new submap

  // GICP verification
  int gicp_max_iterations = 50;
  double gicp_max_correspondence_distance = 1.0;  // m
  double gicp_fitness_threshold = 0.5;      // Max mean squared error for acceptance
  double gicp_min_fitness_threshold = 0.01; // Min fitness (below = suspicious, reject)
  int gicp_min_correspondences = 50;        // Min number of final inliers
  double gicp_max_translation = 2.0;        // Max translation delta from GICP (m)
  double gicp_max_rotation = 0.5;           // Max rotation delta from GICP (rad)
  double overlap_max_distance = 0.35;       // NN distance for overlap/inlier counting (m)
  int min_initial_inliers = 40;             // Initial-guess overlap gate
  double min_initial_overlap_ratio = 0.10;  // Initial source overlap gate
  double min_initial_target_overlap_ratio = 0.15;  // Initial target coverage gate
  double min_final_overlap_ratio = 0.20;    // Final source overlap gate
  double min_final_target_overlap_ratio = 0.05;  // Final target coverage gate
  double min_match_score = 2.0;             // Final score gate to reject weak asymmetric matches
  double heading_max_delta = 2.09;          // Max yaw delta for candidate compatibility (rad)

  // Pose graph
  double loop_information_weight = 100.0;   // Information matrix weight for loop edges
  int pgo_max_iterations = 20;              // Ceres max iterations

  // 2D projection for ground robot
  bool project_to_2d = true;               // Project points to XY plane for matching
  double height_filter_min = -0.1;          // Min z in body frame for projection (m)
  double height_filter_max = 1.5;           // Max z in body frame for projection (m)

  // Debug
  bool enable_debug_log = true;
  std::string debug_csv_path = "";          // Empty = no CSV, set by SlamNode
};

/// Main loop closure pipeline: submap accumulation + spatial proximity + GICP.
class LoopClosureManager {
 public:
  explicit LoopClosureManager(const TofSlamConfig& config,
                               const LoopClosureConfig& lc_config);

  /// Add a keyframe to the manager. Returns submap ID if a new submap was
  /// completed, or -1 if still accumulating.
  int addKeyframe(size_t keyframe_id,
                  const PoseState& pose,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_body);

  /// Check for loop closures against the latest completed submap.
  /// Returns the list of GICP match results (for monitoring).
  /// Accepted matches are automatically added to the pose graph.
  std::vector<GicpMatchResult> detectAndVerifyLoops();

  /// Run pose graph optimization if new loops were added.
  /// Returns true if optimization ran and succeeded.
  bool optimizePoseGraph();

  /// Get the current map-to-odom correction transform.
  Eigen::Matrix4d getMapToOdomCorrection() const;

  /// Get all GICP match results from the most recent detection round.
  std::vector<GicpMatchResult> lastMatchResults() const;

  /// Get cumulative statistics.
  LoopClosureStats getStats() const;

  /// Get a thread-safe snapshot of the pose graph state.
  PoseGraphSnapshot getPoseGraphSnapshot() const;

  /// Get a thread-safe snapshot of keyframe poses.
  std::vector<KeyframePoseSnapshot> getKeyframePoseSnapshots() const;

  /// Get current-best world-frame submaps for visualization/export.
  std::vector<VisualSubmapSnapshot> getVisualSubmapSnapshots() const;

  /// Get a thread-safe snapshot for visualization/debugging.
  LoopClosureDebugSnapshot getDebugSnapshot() const;

  /// Get keyframe manager (for pose updates after PGO)
  KeyframeManager& keyframeManager() { return keyframe_manager_; }
  const KeyframeManager& keyframeManager() const { return keyframe_manager_; }

  /// Get pose graph
  PoseGraph& poseGraph() { return pose_graph_; }
  const PoseGraph& poseGraph() const { return pose_graph_; }

  /// Number of completed submaps
  size_t numSubmaps() const;

  /// Access completed submaps (for map saving)
  const std::vector<SubMap>& getSubmaps() const { return submaps_; }

  /// Check if there are pending (unoptimised) loop closures
  bool hasPendingLoops() const { return pending_loops_ > 0; }

  /// Write GICP diagnostics header to CSV (call once at startup)
  void openDebugCsv(const std::string& path);

  /// Write a batch of GICP results to the debug CSV
  void writeDebugCsv(const std::vector<GicpMatchResult>& results);

  /// Open PGO debug CSV files (summary, edge residuals, node displacements, iterations)
  void openPgoDebugCsvs(const std::string& dump_dir);

 private:
  /// Build a SubMap from accumulated keyframes in the buffer.
  SubMap buildSubMap();

  /// Rebuild a submap in a pose-consistent frame using current-best node poses.
  SubMap buildCurrentBestMatchingSubmapLocked(const SubMap& sm) const;

  /// Compute the current-best centroid of a submap using node poses.
  Eigen::Vector3d computeCurrentBestSubmapCentroidLocked(const SubMap& sm) const;

  /// Find loop closure candidates: submaps whose current-best centroid is
  /// within search_radius but temporally distant.
  std::vector<size_t> findCandidates(const SubMap& query) const;

  /// Run GICP between two submaps rebuilt from current-best node poses.
  GicpMatchResult verifyWithGicp(const SubMap& query,
                                  const SubMap& candidate) const;

  /// Apply 2D height filter + optional XY projection.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterAndProject(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

  /// Voxel downsample a cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilter(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      float voxel_size) const;

  /// Rebuild a submap in world frame using the current best node poses.
  VisualSubmapSnapshot buildVisualSubmapSnapshotLocked(const SubMap& sm) const;

  // Config
  TofSlamConfig config_;
  LoopClosureConfig lc_config_;

  // Core components
  KeyframeManager keyframe_manager_;
  PoseGraph pose_graph_;

  // SubMap management
  std::vector<SubMap> submaps_;
  std::vector<size_t> pending_keyframe_ids_;  // Keyframe IDs not yet in a submap
  size_t next_submap_id_{0};

  // Tracking
  int pending_loops_{0};
  std::vector<GicpMatchResult> last_results_;
  LoopClosureStats stats_;

  // Debug CSV
  bool csv_open_{false};
  std::ofstream csv_file_;

  // PGO debug modules
  PgoDebugLogger pgo_logger_;
  PgoConvergenceMonitor pgo_monitor_;

  mutable std::mutex mutex_;
};

}  // namespace tof_slam

#endif  // TOF_SLAM_BACKEND_LOOP_CLOSURE_MANAGER_HPP_
