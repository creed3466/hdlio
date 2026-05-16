#pragma once

#include <vector>
#include <mutex>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "tof_slam/common/types.hpp"
#include "tof_slam/common/config.hpp"

namespace tof_slam {

/// An edge (factor) in the pose graph
struct PoseGraphEdge {
  size_t from_id;
  size_t to_id;
  Eigen::Matrix4d T_relative;           // measured relative transform
  Eigen::Matrix<double, 6, 6> information;  // information matrix (Sigma^{-1})

  enum Type { WHEEL, SCAN, LOOP } type;
};

/// A node in the pose graph
struct PoseGraphNode {
  size_t id;
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
};

struct PoseGraphOptimizationSummary {
  bool solution_usable{false};
  bool converged{false};
  int iterations{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  std::string termination_type;
  std::string brief_report;
};

/// SE(3) pose graph optimizer using Ceres.
class PoseGraph {
 public:
  explicit PoseGraph(const TofSlamConfig& config);

  /// Add a node to the pose graph
  void addNode(size_t id, const Eigen::Quaterniond& q, const Eigen::Vector3d& p);

  /// Add a relative factor (wheel, scan, or loop)
  void addEdge(const PoseGraphEdge& edge);

  /// Run optimization. Returns true if successful.
  bool optimize(int max_iterations = 10);

  /// Get optimized pose for a node
  bool getOptimizedPose(size_t id, Eigen::Quaterniond& q, Eigen::Vector3d& p) const;

  /// Get the last solver summary for diagnostics
  PoseGraphOptimizationSummary lastOptimizationSummary() const { return last_summary_; }

  /// Get the correction transform (map->odom) after optimization
  Eigen::Matrix4d getMapToOdomCorrection() const;

  /// Get number of nodes and edges
  size_t numNodes() const;
  size_t numEdges() const;

  /// Get all nodes (for visualization/export)
  const std::vector<PoseGraphNode>& getNodes() const { return nodes_; }

  /// Get all edges (for visualization/export)
  const std::vector<PoseGraphEdge>& getEdges() const { return edges_; }

  /// Clear the graph
  void clear();

  /// Thread-safe access
  std::mutex& getMutex() { return mutex_; }

  /// Set an external iteration callback for convergence monitoring
  void setIterationCallback(ceres::IterationCallback* callback) { iteration_callback_ = callback; }

 private:
  TofSlamConfig config_;
  std::vector<PoseGraphNode> nodes_;
  std::vector<PoseGraphEdge> edges_;

  // Optimization data: arrays for Ceres (quaternion wxyz + translation xyz)
  // Stored per node index in nodes_ vector
  std::vector<double> quaternions_;   // 4 doubles per node [w,x,y,z]
  std::vector<double> translations_;  // 3 doubles per node [x,y,z]

  // Pre-optimization pose of the latest node (used for correction computation)
  Eigen::Matrix4d T_latest_before_opt_{Eigen::Matrix4d::Identity()};
  PoseGraphOptimizationSummary last_summary_;
  ceres::IterationCallback* iteration_callback_{nullptr};

  mutable std::mutex mutex_;

  /// Find index into nodes_ for a given node id. Returns -1 if not found.
  int findNodeIndex(size_t id) const;

  /// Check whether an equivalent edge is already present.
  bool hasEquivalentEdge(const PoseGraphEdge& edge) const;

  /// Copy node poses into flat Ceres arrays
  void copyNodesToCeresArrays();

  /// Copy optimized Ceres arrays back into nodes
  void copyCeresArraysToNodes();
};

}  // namespace tof_slam
