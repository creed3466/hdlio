// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// backend_runner.hpp -- Asynchronous backend thread for loop closure.
//
// Receives keyframe data from the frontend, builds submaps, detects loops,
// runs pose graph optimization, and produces T_map_odom corrections.
// All heavy computation runs on a dedicated background thread to preserve
// frontend real-time performance.

#ifndef TOF_SLAM_ROS_BACKEND_RUNNER_HPP_
#define TOF_SLAM_ROS_BACKEND_RUNNER_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <functional>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tof_slam/backend/loop_closure_manager.hpp"
#include "tof_slam/backend/map_saver.hpp"
#include "tof_slam/common/types.hpp"

namespace tof_slam {

/// Queued keyframe data for the backend thread.
struct BackendKeyframe {
  size_t keyframe_id{0};
  PoseState pose;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // Body frame, downsampled
};

/// Latest backend timing snapshot (updated once per processed keyframe).
struct BackendUsageSnapshot {
  std::uint64_t sequence_id{0};
  int queue_depth{0};
  int submap_id{-1};
  int candidates_tested{0};
  int loops_accepted{0};
  bool optimization_ran{false};
  float add_keyframe_ms{0.0f};
  float loop_detect_ms{0.0f};
  float gicp_ms{0.0f};
  float pgo_ms{0.0f};
  float total_ms{0.0f};
};

/// BackendRunner manages the async loop-closure backend thread.
class BackendRunner {
 public:
  /// Callback type for when a new correction is available.
  using CorrectionCallback = std::function<void(const Eigen::Matrix4d&)>;
  using StatusCallback = std::function<void()>;

  BackendRunner(const TofSlamConfig& config,
                const LoopClosureConfig& lc_config);
  ~BackendRunner();

  /// Start the background thread.
  void start();

  /// Stop the background thread (blocks until joined).
  void stop();

  /// Submit a keyframe to the backend (non-blocking, copies data).
  void submitKeyframe(size_t keyframe_id,
                      const PoseState& pose,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  /// Set callback for when T_map_odom correction is updated.
  void setCorrectionCallback(CorrectionCallback cb);

  /// Set callback for backend status/visualization updates.
  void setStatusCallback(StatusCallback cb);

  /// Get the current map-to-odom correction (thread-safe).
  Eigen::Matrix4d getCorrection() const;

  /// Get latest stats (thread-safe).
  LoopClosureStats getStats() const;

  /// Get latest GICP match results (thread-safe).
  std::vector<GicpMatchResult> lastMatchResults() const;

  /// Access the loop closure manager (NOT thread-safe, use with caution).
  LoopClosureManager& manager() { return manager_; }
  const LoopClosureManager& manager() const { return manager_; }

  /// Save the current map to disk (thread-safe: blocks backend while saving).
  /// @param save_dir Output directory path
  /// @param config   Map save configuration
  /// @return Result with success status and statistics
  MapSaveResult saveMap(const std::string& save_dir,
                        const MapSaveConfig& config);

  /// Open PGO debug CSV files in the given directory.
  void openPgoDebugCsvs(const std::string& dump_dir) { manager_.openPgoDebugCsvs(dump_dir); }

  /// Get the latest backend timing snapshot.
  BackendUsageSnapshot latestUsage() const;

 private:
  void runLoop();

  LoopClosureManager manager_;

  // Queue
  std::deque<BackendKeyframe> queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  // Thread
  std::thread thread_;
  std::atomic<bool> running_{false};

  // Correction
  Eigen::Matrix4d correction_{Eigen::Matrix4d::Identity()};
  mutable std::mutex correction_mutex_;
  CorrectionCallback correction_callback_;
  StatusCallback status_callback_;

  // Latest timing snapshot
  BackendUsageSnapshot latest_usage_;
  mutable std::mutex usage_mutex_;
  std::uint64_t usage_sequence_{0};
};

}  // namespace tof_slam

#endif  // TOF_SLAM_ROS_BACKEND_RUNNER_HPP_
