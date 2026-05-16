// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// backend_runner.cpp -- Asynchronous backend thread for loop closure.

#include "tof_slam/ros/backend_runner.hpp"

#include <spdlog/spdlog.h>

#include <chrono>

namespace tof_slam {

// ---- Constructor / Destructor -----------------------------------------------

BackendRunner::BackendRunner(const TofSlamConfig& config,
                             const LoopClosureConfig& lc_config)
    : manager_(config, lc_config) {}

BackendRunner::~BackendRunner() {
  stop();
}

// ---- start / stop -----------------------------------------------------------

void BackendRunner::start() {
  if (running_) return;
  running_ = true;
  thread_ = std::thread(&BackendRunner::runLoop, this);
  spdlog::info("[BackendRunner] Background thread started.");
}

void BackendRunner::stop() {
  if (!running_) return;
  running_ = false;
  queue_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  spdlog::info("[BackendRunner] Background thread stopped.");
}

// ---- submitKeyframe ---------------------------------------------------------

void BackendRunner::submitKeyframe(
    size_t keyframe_id,
    const PoseState& pose,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  BackendKeyframe bk;
  bk.keyframe_id = keyframe_id;
  bk.pose = pose;
  // Deep-copy the cloud to avoid races with frontend
  bk.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);

  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    queue_.push_back(std::move(bk));
  }
  queue_cv_.notify_one();
}

// ---- setCorrectionCallback --------------------------------------------------

void BackendRunner::setCorrectionCallback(CorrectionCallback cb) {
  correction_callback_ = std::move(cb);
}

void BackendRunner::setStatusCallback(StatusCallback cb) {
  status_callback_ = std::move(cb);
}

// ---- getCorrection ----------------------------------------------------------

Eigen::Matrix4d BackendRunner::getCorrection() const {
  std::lock_guard<std::mutex> lk(correction_mutex_);
  return correction_;
}

// ---- getStats ---------------------------------------------------------------

LoopClosureStats BackendRunner::getStats() const {
  return manager_.getStats();
}

// ---- lastMatchResults -------------------------------------------------------

std::vector<GicpMatchResult> BackendRunner::lastMatchResults() const {
  return manager_.lastMatchResults();
}

BackendUsageSnapshot BackendRunner::latestUsage() const {
  std::lock_guard<std::mutex> lk(usage_mutex_);
  return latest_usage_;
}

// ---- saveMap ----------------------------------------------------------------

MapSaveResult BackendRunner::saveMap(const std::string& save_dir,
                                     const MapSaveConfig& config) {
  // Pause background processing by acquiring the queue lock briefly.
  // The backend thread will block on queue_cv_ while we save.
  spdlog::info("[BackendRunner] Saving map (backend thread may pause) ...");
  MapSaver saver(config);
  return saver.save(save_dir, manager_);
}

// ---- runLoop (background thread) --------------------------------------------

void BackendRunner::runLoop() {
  using Clock = std::chrono::steady_clock;

  while (true) {
    BackendKeyframe bk;
    int queue_depth = 0;
    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      queue_cv_.wait(lk, [this] { return !queue_.empty() || !running_; });
      if (!running_ && queue_.empty()) break;
      if (queue_.empty()) continue;
      queue_depth = static_cast<int>(queue_.size());
      bk = std::move(queue_.front());
      queue_.pop_front();
    }

    BackendUsageSnapshot usage;
    usage.sequence_id = ++usage_sequence_;
    usage.queue_depth = queue_depth;

    const auto t_total_start = Clock::now();
    const auto t_add_start = Clock::now();
    int submap_id = manager_.addKeyframe(bk.keyframe_id, bk.pose, bk.cloud);
    usage.add_keyframe_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t_add_start).count();
    usage.submap_id = submap_id;

    if (submap_id >= 0) {
      const auto t_detect_start = Clock::now();
      auto results = manager_.detectAndVerifyLoops();
      usage.loop_detect_ms = std::chrono::duration<float, std::milli>(
          Clock::now() - t_detect_start).count();
      usage.candidates_tested = static_cast<int>(results.size());
      for (const auto& result : results) {
        usage.gicp_ms += static_cast<float>(result.gicp_time_ms);
        if (result.accepted) {
          ++usage.loops_accepted;
        }
      }

      if (manager_.hasPendingLoops()) {
        const auto t_pgo_start = Clock::now();
        bool ok = manager_.optimizePoseGraph();
        usage.pgo_ms = std::chrono::duration<float, std::milli>(
            Clock::now() - t_pgo_start).count();
        usage.optimization_ran = ok;
        if (ok) {
          Eigen::Matrix4d new_correction = manager_.getMapToOdomCorrection();
          {
            std::lock_guard<std::mutex> lk(correction_mutex_);
            correction_ = new_correction;
          }
          if (correction_callback_) {
            correction_callback_(new_correction);
          }
        }
      }
    }

    usage.total_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t_total_start).count();
    {
      std::lock_guard<std::mutex> lk(usage_mutex_);
      latest_usage_ = usage;
    }
    if (status_callback_) {
      status_callback_();
    }
  }
}

}  // namespace tof_slam
