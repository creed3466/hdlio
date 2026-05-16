// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// csv_logger.hpp — CSV trajectory and diagnostics file writer.

#ifndef TOF_SLAM_ROS_CSV_LOGGER_HPP_
#define TOF_SLAM_ROS_CSV_LOGGER_HPP_

#include <Eigen/Dense>
#include <fstream>
#include <mutex>
#include <string>

#include "tof_slam/frontend_w/estimator/lwo_estimator.hpp"

namespace tof_slam {

class CsvLogger {
 public:
  CsvLogger() = default;
  ~CsvLogger();

  // Non-copyable, non-movable (owns file handles + mutexes)
  CsvLogger(const CsvLogger&) = delete;
  CsvLogger& operator=(const CsvLogger&) = delete;

  void open(const std::string& trajectory_csv_path,
            const std::string& dump_path,
            bool enable_diag);

  void close();

  // Write one LWO trajectory row. Thread-safe.
  void write_trajectory(const Eigen::Vector3f& position,
                        const Eigen::Quaternionf& q,
                        double timestamp,
                        const Eigen::Vector3f& rel_odom_pos,
                        const Eigen::Quaternionf& rel_odom_q);

  // Write one LIO trajectory row (no odom). Thread-safe.
  void write_trajectory_lio(const Eigen::Vector3f& position,
                            const Eigen::Quaternionf& q,
                            double timestamp);

  // Write one diagnostics row. Thread-safe.
  void write_diagnostics(const lwo::LwoEstimator::FrameDiagnostics& d);

  // Open usage CSV (called after open()). Thread-safe.
  void open_usage(const std::string& dump_path, const std::string& stem);

  // Write one usage row. Thread-safe.
  void write_usage(const lwo::LwoEstimator::FrameUsage& u);

 private:
  std::ofstream csv_file_;
  std::mutex csv_mutex_;
  int csv_seq_ = 0;

  std::ofstream diag_csv_;
  std::mutex diag_csv_mutex_;

  std::ofstream usage_csv_;
  std::mutex usage_csv_mutex_;
};

}  // namespace tof_slam

#endif  // TOF_SLAM_ROS_CSV_LOGGER_HPP_
