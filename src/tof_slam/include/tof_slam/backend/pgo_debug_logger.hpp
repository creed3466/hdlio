#pragma once
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tof_slam/backend/pose_graph.hpp"
#include "tof_slam/common/se3.hpp"

namespace tof_slam {

class PgoDebugLogger {
public:
  void open(const std::string& dump_dir);
  void close();

  /// Call after each PGO optimization with before/after state
  void logOptimization(
    const std::vector<PoseGraphNode>& before_nodes,
    const std::vector<PoseGraphNode>& after_nodes,
    const std::vector<PoseGraphEdge>& edges,
    const PoseGraphOptimizationSummary& ceres_summary,
    const Eigen::Matrix4d& T_map_odom);

private:
  std::ofstream summary_csv_;
  std::ofstream edge_csv_;
  std::ofstream node_csv_;
  int opt_count_{0};
  Eigen::Matrix4d prev_T_map_odom_{Eigen::Matrix4d::Identity()};
  bool open_{false};
};

}  // namespace tof_slam
