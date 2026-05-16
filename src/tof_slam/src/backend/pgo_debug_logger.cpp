#include "tof_slam/backend/pgo_debug_logger.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>

namespace tof_slam {

namespace {

/// Build id→node lookup
std::unordered_map<size_t, PoseGraphNode> buildNodeMap(
    const std::vector<PoseGraphNode>& nodes) {
  std::unordered_map<size_t, PoseGraphNode> m;
  m.reserve(nodes.size());
  for (const auto& n : nodes) m.emplace(n.id, n);
  return m;
}

/// Compute weighted residual norm and 6D residual (unweighted) from a
/// single edge given a node map.
/// Returns false if either node is missing.
bool computeEdgeResidual(
    const PoseGraphEdge& edge,
    const std::unordered_map<size_t, PoseGraphNode>& node_map,
    double& weighted_norm,
    Eigen::Matrix<double, 6, 1>& xi_out) {
  const auto it_from = node_map.find(edge.from_id);
  const auto it_to   = node_map.find(edge.to_id);
  if (it_from == node_map.end() || it_to == node_map.end()) {
    weighted_norm = 0.0;
    xi_out.setZero();
    return false;
  }

  const Eigen::Matrix4d T_i =
      se3::toTransform(it_from->second.q, it_from->second.p);
  const Eigen::Matrix4d T_j =
      se3::toTransform(it_to->second.q,   it_to->second.p);
  const Eigen::Matrix4d T_error =
      se3::inverseSE3(edge.T_relative) * se3::inverseSE3(T_i) * T_j;

  xi_out = se3::LogSE3(T_error);

  const Eigen::Matrix<double, 6, 6> info =
      0.5 * (edge.information + edge.information.transpose());
  const double weighted_cost =
      std::max(0.0, (xi_out.transpose() * info * xi_out)(0, 0));
  weighted_norm = std::sqrt(weighted_cost);
  return true;
}

double yawFromMatrix4d(const Eigen::Matrix4d& T) {
  return std::atan2(T(1, 0), T(0, 0));
}

double yawFromQuaternion(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<long>(mid), v.end());
  return v[mid];
}

const char* edgeTypeStr(PoseGraphEdge::Type t) {
  switch (t) {
    case PoseGraphEdge::WHEEL: return "WHEEL";
    case PoseGraphEdge::SCAN:  return "SCAN";
    case PoseGraphEdge::LOOP:  return "LOOP";
  }
  return "UNKNOWN";
}

}  // namespace

void PgoDebugLogger::open(const std::string& dump_dir) {
  summary_csv_.open(dump_dir + "/pgo_summary.csv");
  edge_csv_.open(dump_dir + "/pgo_edge_residuals.csv");
  node_csv_.open(dump_dir + "/pgo_node_displacements.csv");

  if (summary_csv_.is_open()) {
    summary_csv_ << "opt_id,num_nodes,num_edges,num_loop_edges,num_wheel_edges,"
                    "ceres_initial_cost,ceres_final_cost,ceres_converged,"
                    "ceres_iterations,ceres_termination,cost_reduction_ratio,"
                    "mean_node_shift_m,max_node_shift_m,median_node_shift_m,"
                    "latest_node_shift_m,mean_loop_res_before,mean_loop_res_after,"
                    "max_loop_res_before,max_loop_res_after,"
                    "mean_wheel_res_before,mean_wheel_res_after,"
                    "T_correction_tx,T_correction_ty,T_correction_tz,"
                    "T_correction_yaw_deg,correction_delta_m,correction_delta_deg\n";
    summary_csv_.flush();
  }

  if (edge_csv_.is_open()) {
    edge_csv_ << "opt_id,edge_idx,from_id,to_id,edge_type,"
                 "residual_before,residual_after,"
                 "res_phi_x,res_phi_y,res_phi_z,"
                 "res_rho_x,res_rho_y,res_rho_z,"
                 "info_trace,T_rel_tx,T_rel_ty,T_rel_tz,T_rel_yaw_deg\n";
    edge_csv_.flush();
  }

  if (node_csv_.is_open()) {
    node_csv_ << "opt_id,node_id,"
                 "before_x,before_y,before_z,before_yaw_deg,"
                 "after_x,after_y,after_z,after_yaw_deg,"
                 "shift_m,yaw_shift_deg\n";
    node_csv_.flush();
  }

  open_ = true;
}

void PgoDebugLogger::close() {
  if (summary_csv_.is_open()) summary_csv_.close();
  if (edge_csv_.is_open())    edge_csv_.close();
  if (node_csv_.is_open())    node_csv_.close();
  open_ = false;
}

void PgoDebugLogger::logOptimization(
    const std::vector<PoseGraphNode>& before_nodes,
    const std::vector<PoseGraphNode>& after_nodes,
    const std::vector<PoseGraphEdge>& edges,
    const PoseGraphOptimizationSummary& ceres_summary,
    const Eigen::Matrix4d& T_map_odom) {

  if (!open_) return;

  const int opt_id = opt_count_++;

  const auto before_map = buildNodeMap(before_nodes);
  const auto after_map  = buildNodeMap(after_nodes);

  // ---- Node displacement CSV ----------------------------------------
  std::vector<double> shifts;
  shifts.reserve(before_nodes.size());
  double latest_shift_m = 0.0;

  for (const auto& nb : before_nodes) {
    const auto it_a = after_map.find(nb.id);
    if (it_a == after_map.end()) continue;

    const double shift = (it_a->second.p - nb.p).norm();
    shifts.push_back(shift);
    latest_shift_m = shift;  // last node = latest

    const double before_yaw = yawFromQuaternion(nb.q) * 180.0 / M_PI;
    const double after_yaw  = yawFromQuaternion(it_a->second.q) * 180.0 / M_PI;
    double yaw_shift = after_yaw - before_yaw;
    // Normalize to [-180, 180]
    while (yaw_shift >  180.0) yaw_shift -= 360.0;
    while (yaw_shift < -180.0) yaw_shift += 360.0;

    if (node_csv_.is_open()) {
      node_csv_ << opt_id << ","
                << nb.id << ","
                << nb.p.x() << "," << nb.p.y() << "," << nb.p.z() << ","
                << before_yaw << ","
                << it_a->second.p.x() << "," << it_a->second.p.y() << ","
                << it_a->second.p.z() << ","
                << after_yaw << ","
                << shift << ","
                << yaw_shift << "\n";
    }
  }
  if (node_csv_.is_open()) node_csv_.flush();

  const double mean_shift = shifts.empty() ? 0.0 :
      std::accumulate(shifts.begin(), shifts.end(), 0.0) /
      static_cast<double>(shifts.size());
  const double max_shift = shifts.empty() ? 0.0 :
      *std::max_element(shifts.begin(), shifts.end());
  const double med_shift  = median(shifts);

  // ---- Edge residual CSV -------------------------------------------
  int num_loop_edges  = 0;
  int num_wheel_edges = 0;
  double loop_res_before_sum  = 0.0, loop_res_after_sum  = 0.0;
  double loop_res_before_max  = 0.0, loop_res_after_max  = 0.0;
  double wheel_res_before_sum = 0.0, wheel_res_after_sum = 0.0;
  int loop_cnt_before = 0, loop_cnt_after = 0;
  int wheel_cnt_before = 0, wheel_cnt_after = 0;

  for (size_t ei = 0; ei < edges.size(); ++ei) {
    const auto& edge = edges[ei];

    double res_before = 0.0;
    Eigen::Matrix<double, 6, 1> xi_before = Eigen::Matrix<double, 6, 1>::Zero();
    computeEdgeResidual(edge, before_map, res_before, xi_before);

    double res_after = 0.0;
    Eigen::Matrix<double, 6, 1> xi_after = Eigen::Matrix<double, 6, 1>::Zero();
    computeEdgeResidual(edge, after_map, res_after, xi_after);

    // Accumulate per-type stats
    if (edge.type == PoseGraphEdge::LOOP) {
      ++num_loop_edges;
      loop_res_before_sum += res_before; ++loop_cnt_before;
      loop_res_after_sum  += res_after;  ++loop_cnt_after;
      loop_res_before_max  = std::max(loop_res_before_max,  res_before);
      loop_res_after_max   = std::max(loop_res_after_max,   res_after);
    } else if (edge.type == PoseGraphEdge::WHEEL) {
      ++num_wheel_edges;
      wheel_res_before_sum += res_before; ++wheel_cnt_before;
      wheel_res_after_sum  += res_after;  ++wheel_cnt_after;
    }

    // LogSE3 convention: [phi(3); rho(3)]  →  phi=xi.head<3>(), rho=xi.tail<3>()
    // (consistent with how the rest of the codebase uses LogSE3)
    const double info_trace = edge.information.trace();
    const double T_rel_yaw  = yawFromMatrix4d(edge.T_relative) * 180.0 / M_PI;

    if (edge_csv_.is_open()) {
      edge_csv_ << opt_id << ","
                << ei << ","
                << edge.from_id << ","
                << edge.to_id << ","
                << edgeTypeStr(edge.type) << ","
                << res_before << ","
                << res_after << ","
                // phi = rotation components (first 3 of LogSE3)
                << xi_after(0) << "," << xi_after(1) << "," << xi_after(2) << ","
                // rho = translation components (last 3 of LogSE3)
                << xi_after(3) << "," << xi_after(4) << "," << xi_after(5) << ","
                << info_trace << ","
                << edge.T_relative(0, 3) << ","
                << edge.T_relative(1, 3) << ","
                << edge.T_relative(2, 3) << ","
                << T_rel_yaw << "\n";
    }
  }
  if (edge_csv_.is_open()) edge_csv_.flush();

  const double mean_loop_res_before  = loop_cnt_before  > 0 ?
      loop_res_before_sum  / static_cast<double>(loop_cnt_before)  : 0.0;
  const double mean_loop_res_after   = loop_cnt_after   > 0 ?
      loop_res_after_sum   / static_cast<double>(loop_cnt_after)   : 0.0;
  const double mean_wheel_res_before = wheel_cnt_before > 0 ?
      wheel_res_before_sum / static_cast<double>(wheel_cnt_before) : 0.0;
  const double mean_wheel_res_after  = wheel_cnt_after  > 0 ?
      wheel_res_after_sum  / static_cast<double>(wheel_cnt_after)  : 0.0;

  // ---- T_map_odom stats --------------------------------------------
  const double tx  = T_map_odom(0, 3);
  const double ty  = T_map_odom(1, 3);
  const double tz  = T_map_odom(2, 3);
  const double yaw = yawFromMatrix4d(T_map_odom) * 180.0 / M_PI;

  // Delta from previous correction
  const Eigen::Matrix4d T_delta =
      se3::inverseSE3(prev_T_map_odom_) * T_map_odom;
  const double delta_t   = T_delta.block<3,1>(0,3).norm();
  const Eigen::Matrix<double, 6, 1> xi_delta = se3::LogSE3(T_delta);
  const double delta_deg = xi_delta.head<3>().norm() * 180.0 / M_PI;

  prev_T_map_odom_ = T_map_odom;

  // ---- cost reduction ratio ----------------------------------------
  const double cost_reduction_ratio =
      (ceres_summary.initial_cost > 1e-12) ?
      (ceres_summary.initial_cost - ceres_summary.final_cost) /
       ceres_summary.initial_cost : 0.0;

  // ---- Summary CSV -------------------------------------------------
  if (summary_csv_.is_open()) {
    summary_csv_ << opt_id << ","
                 << before_nodes.size() << ","
                 << edges.size() << ","
                 << num_loop_edges << ","
                 << num_wheel_edges << ","
                 << ceres_summary.initial_cost << ","
                 << ceres_summary.final_cost << ","
                 << (ceres_summary.converged ? 1 : 0) << ","
                 << ceres_summary.iterations << ","
                 << "\"" << ceres_summary.termination_type << "\"" << ","
                 << cost_reduction_ratio << ","
                 << mean_shift << ","
                 << max_shift << ","
                 << med_shift << ","
                 << latest_shift_m << ","
                 << mean_loop_res_before << ","
                 << mean_loop_res_after << ","
                 << loop_res_before_max << ","
                 << loop_res_after_max << ","
                 << mean_wheel_res_before << ","
                 << mean_wheel_res_after << ","
                 << tx << ","
                 << ty << ","
                 << tz << ","
                 << yaw << ","
                 << delta_t << ","
                 << delta_deg << "\n";
    summary_csv_.flush();
  }
}

}  // namespace tof_slam
