#include "tof_slam/backend/pose_graph.hpp"
#include "tof_slam/common/se3.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cassert>

namespace tof_slam {

/// Ceres cost functor for relative SE(3) factors.
/// Residual: r = sqrt_info * LogSE3(T_measured^{-1} * T_i^{-1} * T_j)
/// Convention: xi = [phi(3); rho(3)], quaternion [w,x,y,z] for Ceres.
class SE3RelativeCostFunction
    : public ceres::SizedCostFunction<6, 4, 3, 4, 3> {
 public:
  SE3RelativeCostFunction(const Eigen::Matrix4d& T_measured,
                          const Eigen::Matrix<double, 6, 6>& sqrt_info,
                          double numeric_diff_eps)
      : T_measured_inv_(se3::inverseSE3(T_measured)),
        sqrt_info_(sqrt_info),
        numeric_diff_eps_(numeric_diff_eps) {}

  bool Evaluate(double const* const* parameters,
                double* residuals,
                double** jacobians) const override {
    // --- Extract node i: quaternion [w,x,y,z], translation [x,y,z] --------
    const Eigen::Map<const Eigen::Quaterniond> q_i(parameters[0]);  // Eigen stores [x,y,z,w] internally but Map reads in storage order
    const Eigen::Map<const Eigen::Vector3d> p_i(parameters[1]);

    const Eigen::Map<const Eigen::Quaterniond> q_j(parameters[2]);
    const Eigen::Map<const Eigen::Vector3d> p_j(parameters[3]);

    // NOTE: Ceres QuaternionManifold stores [w,x,y,z].
    // Eigen::Quaterniond internal storage is [x,y,z,w].
    // We need to convert from Ceres [w,x,y,z] to Eigen.
    Eigen::Quaterniond qi_eigen(parameters[0][0], parameters[0][1],
                                parameters[0][2], parameters[0][3]);
    Eigen::Quaterniond qj_eigen(parameters[2][0], parameters[2][1],
                                parameters[2][2], parameters[2][3]);

    qi_eigen.normalize();
    qj_eigen.normalize();

    // Build 4x4 transforms
    Eigen::Matrix4d T_i = se3::toTransform(qi_eigen, p_i);
    Eigen::Matrix4d T_j = se3::toTransform(qj_eigen, p_j);

    // T_error = T_measured^{-1} * T_i^{-1} * T_j
    Eigen::Matrix4d T_i_inv = se3::inverseSE3(T_i);
    Eigen::Matrix4d T_error = T_measured_inv_ * T_i_inv * T_j;

    // Log map -> 6-vector [phi; rho]
    Eigen::Matrix<double, 6, 1> xi_err = se3::LogSE3(T_error);

    // Weighted residual
    Eigen::Map<Eigen::Matrix<double, 6, 1>> res(residuals);
    res = sqrt_info_ * xi_err;

    // Analytic Jacobians are complex for SE(3). Use numeric differentiation
    // by returning true with jacobians = nullptr when requested.
    // Ceres will fall back to numeric differentiation if we don't provide them.
    if (jacobians) {
      // Use finite-difference Jacobians via Ceres' NumericDiffMethod.
      // We implement a first-order forward-difference here for correctness.
      const double kEps = numeric_diff_eps_;

      // Helper lambda: evaluate residual for perturbed parameters
      auto eval_residual = [&](int block, int idx, double delta,
                               Eigen::Matrix<double, 6, 1>& r) {
        // Copy parameters
        double q_i_pert[4], p_i_pert[3], q_j_pert[4], p_j_pert[3];
        std::copy(parameters[0], parameters[0] + 4, q_i_pert);
        std::copy(parameters[1], parameters[1] + 3, p_i_pert);
        std::copy(parameters[2], parameters[2] + 4, q_j_pert);
        std::copy(parameters[3], parameters[3] + 3, p_j_pert);

        double* blocks[4] = {q_i_pert, p_i_pert, q_j_pert, p_j_pert};
        blocks[block][idx] += delta;

        // Re-normalize quaternion if perturbing quaternion block
        if (block == 0 || block == 2) {
          double norm = std::sqrt(blocks[block][0] * blocks[block][0] +
                                  blocks[block][1] * blocks[block][1] +
                                  blocks[block][2] * blocks[block][2] +
                                  blocks[block][3] * blocks[block][3]);
          for (int k = 0; k < 4; ++k) blocks[block][k] /= norm;
        }

        Eigen::Quaterniond qi_p(blocks[0][0], blocks[0][1],
                                blocks[0][2], blocks[0][3]);
        Eigen::Quaterniond qj_p(blocks[2][0], blocks[2][1],
                                blocks[2][2], blocks[2][3]);
        qi_p.normalize();
        qj_p.normalize();

        Eigen::Map<const Eigen::Vector3d> pi_p(blocks[1]);
        Eigen::Map<const Eigen::Vector3d> pj_p(blocks[3]);

        Eigen::Matrix4d Ti = se3::toTransform(qi_p, pi_p);
        Eigen::Matrix4d Tj = se3::toTransform(qj_p, pj_p);
        Eigen::Matrix4d Terr = T_measured_inv_ * se3::inverseSE3(Ti) * Tj;
        Eigen::Matrix<double, 6, 1> xi = se3::LogSE3(Terr);
        r = sqrt_info_ * xi;
      };

      const int block_sizes[4] = {4, 3, 4, 3};
      for (int b = 0; b < 4; ++b) {
        if (jacobians[b]) {
          Eigen::Map<Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::RowMajor>>
              J(jacobians[b], 6, block_sizes[b]);
          for (int col = 0; col < block_sizes[b]; ++col) {
            Eigen::Matrix<double, 6, 1> r_plus;
            eval_residual(b, col, kEps, r_plus);
            J.col(col) = (r_plus - res) / kEps;
          }
        }
      }
    }

    return true;
  }

 private:
  Eigen::Matrix4d T_measured_inv_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
  double numeric_diff_eps_{1e-6};
};

PoseGraph::PoseGraph(const TofSlamConfig& config) : config_(config) {}

void PoseGraph::addNode(size_t id,
                        const Eigen::Quaterniond& q,
                        const Eigen::Vector3d& p) {
  // Check for duplicate id
  if (findNodeIndex(id) >= 0) {
    // Update existing node
    int idx = findNodeIndex(id);
    nodes_[idx].q = q;
    nodes_[idx].p = p;
    return;
  }

  PoseGraphNode node;
  node.id = id;
  node.q = q.normalized();
  node.p = p;
  nodes_.push_back(node);

  // Expand Ceres arrays
  // Quaternion in Ceres convention [w, x, y, z]
  quaternions_.push_back(node.q.w());
  quaternions_.push_back(node.q.x());
  quaternions_.push_back(node.q.y());
  quaternions_.push_back(node.q.z());

  translations_.push_back(p.x());
  translations_.push_back(p.y());
  translations_.push_back(p.z());
}

void PoseGraph::addEdge(const PoseGraphEdge& edge) {
  if (edge.from_id == edge.to_id) {
    return;
  }

  if (hasEquivalentEdge(edge)) {
    return;
  }

  edges_.push_back(edge);
}

int PoseGraph::findNodeIndex(size_t id) const {
  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].id == id) return static_cast<int>(i);
  }
  return -1;
}

bool PoseGraph::hasEquivalentEdge(const PoseGraphEdge& edge) const {
  return std::any_of(edges_.begin(), edges_.end(),
                     [&](const PoseGraphEdge& existing) {
                       return existing.from_id == edge.from_id &&
                              existing.to_id == edge.to_id &&
                              existing.type == edge.type;
                     });
}

void PoseGraph::copyNodesToCeresArrays() {
  quaternions_.resize(nodes_.size() * 4);
  translations_.resize(nodes_.size() * 3);
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const auto& q = nodes_[i].q.normalized();
    quaternions_[i * 4 + 0] = q.w();
    quaternions_[i * 4 + 1] = q.x();
    quaternions_[i * 4 + 2] = q.y();
    quaternions_[i * 4 + 3] = q.z();
    translations_[i * 3 + 0] = nodes_[i].p.x();
    translations_[i * 3 + 1] = nodes_[i].p.y();
    translations_[i * 3 + 2] = nodes_[i].p.z();
  }
}

void PoseGraph::copyCeresArraysToNodes() {
  for (size_t i = 0; i < nodes_.size(); ++i) {
    nodes_[i].q = Eigen::Quaterniond(quaternions_[i * 4 + 0],   // w
                                     quaternions_[i * 4 + 1],   // x
                                     quaternions_[i * 4 + 2],   // y
                                     quaternions_[i * 4 + 3]);  // z
    nodes_[i].q.normalize();
    nodes_[i].p = Eigen::Vector3d(translations_[i * 3 + 0],
                                  translations_[i * 3 + 1],
                                  translations_[i * 3 + 2]);
  }
}

bool PoseGraph::optimize(int max_iterations) {
  if (nodes_.size() < 2 || edges_.empty()) {
    return false;
  }

  // Save latest node pose before optimization (for correction computation)
  const auto& latest = nodes_.back();
  T_latest_before_opt_ = se3::toTransform(latest.q, latest.p);

  // Sync node data into flat arrays
  copyNodesToCeresArrays();

  // Build Ceres problem
  ceres::Problem problem;

  // Add parameter blocks for each node
  for (size_t i = 0; i < nodes_.size(); ++i) {
    double* q_ptr = &quaternions_[i * 4];
    double* p_ptr = &translations_[i * 3];

    problem.AddParameterBlock(q_ptr, 4, new ceres::QuaternionManifold());
    problem.AddParameterBlock(p_ptr, 3);
  }

  // Fix the first node to remove gauge freedom
  int first_idx = 0;
  problem.SetParameterBlockConstant(&quaternions_[first_idx * 4]);
  problem.SetParameterBlockConstant(&translations_[first_idx * 3]);

  // Add residual blocks for each edge
  for (const auto& edge : edges_) {
    int idx_i = findNodeIndex(edge.from_id);
    int idx_j = findNodeIndex(edge.to_id);
    if (idx_i < 0 || idx_j < 0) {
      continue;  // skip edges with unknown nodes
    }

    // Compute square root of information matrix via Cholesky (LLT)
    // info = L * L^T => sqrt_info = L^T
    Eigen::Matrix<double, 6, 6> info = edge.information;
    // Ensure symmetry
    info = 0.5 * (info + info.transpose());
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(info);
    if (llt.info() != Eigen::Success) {
      // If Cholesky fails, fall back to identity weighting
      Eigen::Matrix<double, 6, 6> sqrt_info =
          Eigen::Matrix<double, 6, 6>::Identity();
      ceres::CostFunction* cost =
          new SE3RelativeCostFunction(edge.T_relative, sqrt_info, config_.pgo_numeric_diff_eps);
      problem.AddResidualBlock(
          cost, nullptr,
          &quaternions_[idx_i * 4], &translations_[idx_i * 3],
          &quaternions_[idx_j * 4], &translations_[idx_j * 3]);
    } else {
      // sqrt_info = L^T  (upper triangular)
      Eigen::Matrix<double, 6, 6> sqrt_info = llt.matrixL().transpose();

      ceres::CostFunction* cost =
          new SE3RelativeCostFunction(edge.T_relative, sqrt_info, config_.pgo_numeric_diff_eps);

      ceres::LossFunction* loss = nullptr;
      if (edge.type == PoseGraphEdge::LOOP && config_.pgo_use_loop_huber) {
        loss = new ceres::HuberLoss(config_.huber_delta);
      }

      problem.AddResidualBlock(
          cost, loss,
          &quaternions_[idx_i * 4], &translations_[idx_i * 3],
          &quaternions_[idx_j * 4], &translations_[idx_j * 3]);
    }
  }

  // Configure solver
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = max_iterations;
  options.num_threads = 4;
  options.minimizer_progress_to_stdout = false;
  options.function_tolerance = 1e-6;
  options.gradient_tolerance = 1e-10;
  options.parameter_tolerance = 1e-8;

  if (iteration_callback_) {
    options.callbacks.push_back(iteration_callback_);
    options.update_state_every_iteration = true;
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  last_summary_.solution_usable = summary.IsSolutionUsable();
  last_summary_.converged = summary.termination_type == ceres::CONVERGENCE;
  last_summary_.iterations = static_cast<int>(summary.iterations.size());
  last_summary_.initial_cost = summary.initial_cost;
  last_summary_.final_cost = summary.final_cost;
  last_summary_.termination_type = summary.BriefReport();
  last_summary_.brief_report = summary.FullReport();

  // Copy results back
  copyCeresArraysToNodes();

  return summary.IsSolutionUsable();
}

bool PoseGraph::getOptimizedPose(size_t id,
                                 Eigen::Quaterniond& q,
                                 Eigen::Vector3d& p) const {
  int idx = findNodeIndex(id);
  if (idx < 0) return false;
  q = nodes_[idx].q;
  p = nodes_[idx].p;
  return true;
}

Eigen::Matrix4d PoseGraph::getMapToOdomCorrection() const {
  if (nodes_.empty()) {
    return Eigen::Matrix4d::Identity();
  }

  // Correction = T_optimized_latest * T_original_latest^{-1}
  // This transforms from the odom frame (where the original pose lived)
  // to the map frame (where the optimized pose lives).
  const auto& latest = nodes_.back();
  Eigen::Matrix4d T_optimized = se3::toTransform(latest.q, latest.p);
  Eigen::Matrix4d T_original_inv = se3::inverseSE3(T_latest_before_opt_);

  return T_optimized * T_original_inv;
}

size_t PoseGraph::numNodes() const {
  return nodes_.size();
}

size_t PoseGraph::numEdges() const {
  return edges_.size();
}

void PoseGraph::clear() {
  nodes_.clear();
  edges_.clear();
  quaternions_.clear();
  translations_.clear();
  T_latest_before_opt_ = Eigen::Matrix4d::Identity();
}

}  // namespace tof_slam
