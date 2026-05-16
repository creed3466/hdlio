// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// loop_closure_manager.cpp -- Spatial-proximity + GICP loop closure pipeline.

#include "tof_slam/backend/loop_closure_manager.hpp"
#include "tof_slam/common/se3.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <numeric>
#include <unordered_map>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

#include <spdlog/spdlog.h>

namespace tof_slam {

namespace {

struct OverlapStats {
  int inlier_count{0};
  double source_overlap_ratio{0.0};
  double target_overlap_ratio{0.0};
  double median_distance{0.0};
  double p90_distance{0.0};
};

struct GraphDiagnostics {
  double cost_before{0.0};
  double cost_after{0.0};
  double mean_node_shift_m{0.0};
  double max_node_shift_m{0.0};
  double latest_node_shift_m{0.0};
  double mean_loop_residual_before{0.0};
  double mean_loop_residual_after{0.0};
  double max_loop_residual_before{0.0};
  double max_loop_residual_after{0.0};
};

double normalizeAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double yawFromQuaternion(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

double yawDeltaAbs(const Eigen::Quaterniond& qa, const Eigen::Quaterniond& qb) {
  return std::abs(normalizeAngle(yawFromQuaternion(qa) - yawFromQuaternion(qb)));
}

OverlapStats computeOverlapStats(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Eigen::Matrix4d& T_source_to_target,
    double max_nn_distance) {
  OverlapStats stats;
  if (!source || !target || source->empty() || target->empty() || max_nn_distance <= 0.0) {
    return stats;
  }

  pcl::PointCloud<pcl::PointXYZ> source_in_target;
  pcl::transformPointCloud(*source, source_in_target, T_source_to_target.cast<float>());

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(target);

  const float max_sq_dist = static_cast<float>(max_nn_distance * max_nn_distance);
  std::vector<float> distances;
  distances.reserve(source_in_target.size());
  std::vector<int> nn_index(1);
  std::vector<float> nn_sq_dist(1);
  std::vector<uint8_t> target_hits(target->size(), 0);

  for (const auto& pt : source_in_target.points) {
    if (kdtree.nearestKSearch(pt, 1, nn_index, nn_sq_dist) <= 0) {
      continue;
    }
    if (nn_sq_dist[0] <= max_sq_dist) {
      ++stats.inlier_count;
      distances.push_back(std::sqrt(nn_sq_dist[0]));
      target_hits[static_cast<size_t>(nn_index[0])] = 1;
    }
  }

  stats.source_overlap_ratio =
      static_cast<double>(stats.inlier_count) / static_cast<double>(source_in_target.size());
  const int unique_target_hits =
      static_cast<int>(std::count(target_hits.begin(), target_hits.end(), static_cast<uint8_t>(1)));
  stats.target_overlap_ratio =
      target->empty() ? 0.0 : static_cast<double>(unique_target_hits) / static_cast<double>(target->size());

  if (!distances.empty()) {
    std::sort(distances.begin(), distances.end());
    stats.median_distance = distances[distances.size() / 2];
    const size_t p90_idx = static_cast<size_t>(0.9 * static_cast<double>(distances.size() - 1));
    stats.p90_distance = distances[p90_idx];
  }

  return stats;
}

std::unordered_map<size_t, PoseGraphNode> buildNodeMap(
    const std::vector<PoseGraphNode>& nodes) {
  std::unordered_map<size_t, PoseGraphNode> map;
  map.reserve(nodes.size());
  for (const auto& node : nodes) {
    map.emplace(node.id, node);
  }
  return map;
}

bool computeEdgeResidualMetrics(
    const PoseGraphEdge& edge,
    const std::unordered_map<size_t, PoseGraphNode>& node_map,
    double& weighted_cost,
    double& weighted_norm) {
  const auto it_from = node_map.find(edge.from_id);
  const auto it_to = node_map.find(edge.to_id);
  if (it_from == node_map.end() || it_to == node_map.end()) {
    weighted_cost = 0.0;
    weighted_norm = 0.0;
    return false;
  }

  const Eigen::Matrix4d T_i =
      se3::toTransform(it_from->second.q, it_from->second.p);
  const Eigen::Matrix4d T_j =
      se3::toTransform(it_to->second.q, it_to->second.p);
  const Eigen::Matrix4d T_error =
      se3::inverseSE3(edge.T_relative) * se3::inverseSE3(T_i) * T_j;
  const Eigen::Matrix<double, 6, 1> xi_err = se3::LogSE3(T_error);
  const Eigen::Matrix<double, 6, 6> info =
      0.5 * (edge.information + edge.information.transpose());
  weighted_cost = std::max(0.0, (xi_err.transpose() * info * xi_err)(0, 0));
  weighted_norm = std::sqrt(weighted_cost);
  return true;
}

GraphDiagnostics computeGraphDiagnostics(
    const std::vector<PoseGraphNode>& before_nodes,
    const std::vector<PoseGraphNode>& after_nodes,
    const std::vector<PoseGraphEdge>& edges) {
  GraphDiagnostics diag;
  const auto before_map = buildNodeMap(before_nodes);
  const auto after_map = buildNodeMap(after_nodes);

  double shift_sum = 0.0;
  int shift_count = 0;
  for (const auto& node_before : before_nodes) {
    const auto it_after = after_map.find(node_before.id);
    if (it_after == after_map.end()) {
      continue;
    }
    const double shift = (it_after->second.p - node_before.p).norm();
    shift_sum += shift;
    ++shift_count;
    diag.max_node_shift_m = std::max(diag.max_node_shift_m, shift);
  }
  if (shift_count > 0) {
    diag.mean_node_shift_m = shift_sum / static_cast<double>(shift_count);
  }
  if (!before_nodes.empty()) {
    const auto latest_id = before_nodes.back().id;
    const auto it_before = before_map.find(latest_id);
    const auto it_after = after_map.find(latest_id);
    if (it_before != before_map.end() && it_after != after_map.end()) {
      diag.latest_node_shift_m = (it_after->second.p - it_before->second.p).norm();
    }
  }

  double loop_res_sum_before = 0.0;
  double loop_res_sum_after = 0.0;
  int loop_count_before = 0;
  int loop_count_after = 0;
  for (const auto& edge : edges) {
    double cost_before = 0.0;
    double norm_before = 0.0;
    if (computeEdgeResidualMetrics(edge, before_map, cost_before, norm_before)) {
      diag.cost_before += cost_before;
      if (edge.type == PoseGraphEdge::LOOP) {
        loop_res_sum_before += norm_before;
        ++loop_count_before;
        diag.max_loop_residual_before = std::max(diag.max_loop_residual_before, norm_before);
      }
    }

    double cost_after = 0.0;
    double norm_after = 0.0;
    if (computeEdgeResidualMetrics(edge, after_map, cost_after, norm_after)) {
      diag.cost_after += cost_after;
      if (edge.type == PoseGraphEdge::LOOP) {
        loop_res_sum_after += norm_after;
        ++loop_count_after;
        diag.max_loop_residual_after = std::max(diag.max_loop_residual_after, norm_after);
      }
    }
  }

  if (loop_count_before > 0) {
    diag.mean_loop_residual_before =
        loop_res_sum_before / static_cast<double>(loop_count_before);
  }
  if (loop_count_after > 0) {
    diag.mean_loop_residual_after =
        loop_res_sum_after / static_cast<double>(loop_count_after);
  }
  return diag;
}

}  // namespace

LoopClosureManager::LoopClosureManager(const TofSlamConfig& config,
                                       const LoopClosureConfig& lc_config)
    : config_(config),
      lc_config_(lc_config),
      keyframe_manager_(config),
      pose_graph_(config) {
  pose_graph_.setIterationCallback(&pgo_monitor_);
}

void LoopClosureManager::openPgoDebugCsvs(const std::string& dump_dir) {
  pgo_logger_.open(dump_dir);
  pgo_monitor_.openCsv(dump_dir + "/pgo_iterations.csv");
}

int LoopClosureManager::addKeyframe(
    size_t keyframe_id,
    const PoseState& pose,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_body) {
  std::lock_guard<std::mutex> lk(mutex_);

  const size_t expected_id = keyframe_manager_.size();
  if (keyframe_id != expected_id) {
    spdlog::error(
        "[LC] Keyframe ID mismatch: received {} but expected {}",
        keyframe_id, expected_id);
    return -1;
  }

  // Use a zero covariance placeholder -- actual covariance not needed for PGO here
  Eigen::Matrix<double, 6, 6> zero_cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;

  size_t kf_id = keyframe_manager_.addKeyframe(
      pose, cloud_body, zero_cov, zero_cov);

  // Add PoseGraph node
  pose_graph_.addNode(kf_id, pose.q_wb, pose.p_wb);

  // Add odometry edge from previous keyframe
  if (kf_id > 0) {
    const auto& prev_kf = keyframe_manager_.getKeyframe(kf_id - 1);
    const auto& curr_kf = keyframe_manager_.getKeyframe(kf_id);

    Eigen::Matrix4d T_prev = se3::toTransform(prev_kf.state.q_wb, prev_kf.state.p_wb);
    Eigen::Matrix4d T_curr = se3::toTransform(curr_kf.state.q_wb, curr_kf.state.p_wb);
    Eigen::Matrix4d T_rel = se3::inverseSE3(T_prev) * T_curr;

    PoseGraphEdge odom_edge;
    odom_edge.from_id = kf_id - 1;
    odom_edge.to_id = kf_id;
    odom_edge.T_relative = T_rel;
    odom_edge.information = Eigen::Matrix<double, 6, 6>::Identity() * 100.0;
    odom_edge.type = PoseGraphEdge::WHEEL;
    pose_graph_.addEdge(odom_edge);
  }

  pending_keyframe_ids_.push_back(kf_id);

  // Check if we have enough keyframes for a new submap
  if (static_cast<int>(pending_keyframe_ids_.size()) >=
      lc_config_.submap_keyframe_count) {
    SubMap sm = buildSubMap();
    submaps_.push_back(std::move(sm));
    stats_.total_submaps = static_cast<int>(submaps_.size());

    if (lc_config_.enable_debug_log) {
      spdlog::info("[LC] SubMap {} built: kf[{}-{}] points={} centroid=[{:.2f},{:.2f},{:.2f}]",
                   submaps_.back().id,
                   submaps_.back().first_keyframe_id,
                   submaps_.back().last_keyframe_id,
                   submaps_.back().total_points,
                   submaps_.back().centroid.x(),
                   submaps_.back().centroid.y(),
                   submaps_.back().centroid.z());
    }

    return static_cast<int>(submaps_.back().id);
  }

  return -1;
}

SubMap LoopClosureManager::buildSubMap() {
  SubMap sm;
  sm.id = next_submap_id_++;
  sm.first_keyframe_id = pending_keyframe_ids_.front();
  sm.last_keyframe_id = pending_keyframe_ids_.back();
  sm.num_keyframes = pending_keyframe_ids_.size();
  sm.keyframe_ids = pending_keyframe_ids_;

  // Reference pose: first keyframe in the batch
  const auto& ref_kf = keyframe_manager_.getKeyframe(sm.first_keyframe_id);
  sm.reference_pose = ref_kf.state;

  Eigen::Matrix4d T_ref = se3::toTransform(ref_kf.state.q_wb, ref_kf.state.p_wb);
  Eigen::Matrix4d T_ref_inv = se3::inverseSE3(T_ref);

  // Accumulate centroid
  Eigen::Vector3d centroid_sum = Eigen::Vector3d::Zero();

  for (size_t kf_id : pending_keyframe_ids_) {
    const auto& kf = keyframe_manager_.getKeyframe(kf_id);
    if (!kf.cloud || kf.cloud->empty()) continue;

    centroid_sum += kf.state.p_wb;

    // Transform cloud from body frame to world, then to reference frame
    Eigen::Matrix4d T_kf = se3::toTransform(kf.state.q_wb, kf.state.p_wb);
    Eigen::Matrix4d T_kf_in_ref = T_ref_inv * T_kf;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*kf.cloud, *transformed,
                             T_kf_in_ref.cast<float>());

    *sm.cloud += *transformed;
  }

  sm.centroid = centroid_sum / static_cast<double>(pending_keyframe_ids_.size());
  sm.total_points = sm.cloud->size();

  // Voxel downsample the accumulated cloud
  if (lc_config_.submap_voxel_size > 0.0) {
    sm.cloud = voxelFilter(sm.cloud,
                           static_cast<float>(lc_config_.submap_voxel_size));
    sm.total_points = sm.cloud->size();
  }

  // Clear pending buffer
  pending_keyframe_ids_.clear();

  return sm;
}

SubMap LoopClosureManager::buildCurrentBestMatchingSubmapLocked(
    const SubMap& sm) const {
  SubMap rebuilt;
  rebuilt.id = sm.id;
  rebuilt.first_keyframe_id = sm.first_keyframe_id;
  rebuilt.last_keyframe_id = sm.last_keyframe_id;
  rebuilt.num_keyframes = sm.num_keyframes;
  rebuilt.keyframe_ids = sm.keyframe_ids;

  if (sm.keyframe_ids.empty()) {
    return rebuilt;
  }

  const auto& ref_kf = keyframe_manager_.getKeyframe(sm.first_keyframe_id);
  rebuilt.reference_pose = ref_kf.state;
  pose_graph_.getOptimizedPose(sm.first_keyframe_id,
                               rebuilt.reference_pose.q_wb,
                               rebuilt.reference_pose.p_wb);

  const Eigen::Matrix4d T_ref =
      se3::toTransform(rebuilt.reference_pose.q_wb, rebuilt.reference_pose.p_wb);
  const Eigen::Matrix4d T_ref_inv = se3::inverseSE3(T_ref);

  Eigen::Vector3d centroid_sum = Eigen::Vector3d::Zero();
  size_t centroid_count = 0;

  for (size_t kf_id : sm.keyframe_ids) {
    if (kf_id >= keyframe_manager_.size()) {
      continue;
    }

    const auto& kf = keyframe_manager_.getKeyframe(kf_id);
    Eigen::Quaterniond q = kf.state.q_wb;
    Eigen::Vector3d p = kf.state.p_wb;
    pose_graph_.getOptimizedPose(kf_id, q, p);

    centroid_sum += p;
    ++centroid_count;

    if (!kf.cloud || kf.cloud->empty()) {
      continue;
    }

    const Eigen::Matrix4d T_kf = se3::toTransform(q, p);
    const Eigen::Matrix4d T_kf_in_ref = T_ref_inv * T_kf;

    pcl::PointCloud<pcl::PointXYZ> transformed;
    pcl::transformPointCloud(*kf.cloud, transformed, T_kf_in_ref.cast<float>());
    *rebuilt.cloud += transformed;
  }

  if (centroid_count > 0) {
    rebuilt.centroid = centroid_sum / static_cast<double>(centroid_count);
  } else {
    rebuilt.centroid = rebuilt.reference_pose.p_wb;
  }

  if (lc_config_.submap_voxel_size > 0.0 && !rebuilt.cloud->empty()) {
    rebuilt.cloud = voxelFilter(rebuilt.cloud,
                                static_cast<float>(lc_config_.submap_voxel_size));
  }
  rebuilt.total_points = rebuilt.cloud->size();
  return rebuilt;
}

Eigen::Vector3d LoopClosureManager::computeCurrentBestSubmapCentroidLocked(
    const SubMap& sm) const {
  if (sm.keyframe_ids.empty()) {
    return sm.centroid;
  }

  Eigen::Vector3d centroid_sum = Eigen::Vector3d::Zero();
  size_t centroid_count = 0;
  for (size_t kf_id : sm.keyframe_ids) {
    if (kf_id >= keyframe_manager_.size()) {
      continue;
    }

    const auto& kf = keyframe_manager_.getKeyframe(kf_id);
    Eigen::Quaterniond q = kf.state.q_wb;
    Eigen::Vector3d p = kf.state.p_wb;
    pose_graph_.getOptimizedPose(kf_id, q, p);
    centroid_sum += p;
    ++centroid_count;
  }

  if (centroid_count == 0) {
    return sm.centroid;
  }
  return centroid_sum / static_cast<double>(centroid_count);
}

std::vector<GicpMatchResult> LoopClosureManager::detectAndVerifyLoops() {
  std::lock_guard<std::mutex> lk(mutex_);

  last_results_.clear();

  if (submaps_.size() < 2) {
    return last_results_;
  }

  const SubMap& query = submaps_.back();
  const auto candidates = findCandidates(query);

  if (lc_config_.enable_debug_log && !candidates.empty()) {
    spdlog::info("[LC] SubMap {} has {} loop candidates",
                 query.id, candidates.size());
  }

  last_results_.reserve(candidates.size());
  int best_result_index = -1;
  double best_match_score = -std::numeric_limits<double>::infinity();

  for (size_t cand_idx : candidates) {
    const SubMap& candidate = submaps_[cand_idx];
    auto result = verifyWithGicp(query, candidate);

    stats_.total_candidates_tested++;
    if (result.converged) {
      stats_.total_gicp_converged++;
    }
    if (result.verification_passed && result.match_score > best_match_score) {
      best_match_score = result.match_score;
      best_result_index = static_cast<int>(last_results_.size());
    }

    last_results_.push_back(std::move(result));
  }

  if (best_result_index >= 0) {
    auto& best = last_results_[static_cast<size_t>(best_result_index)];
    best.accepted = true;
    best.reject_reason = "accepted_best_candidate";

    PoseGraphEdge loop_edge;
    loop_edge.from_id = best.match_keyframe_id;
    loop_edge.to_id = best.query_keyframe_id;
    loop_edge.T_relative = best.T_relative;

    const double overlap_weight = std::max(
        0.05,
        std::min(best.source_overlap_ratio, best.target_overlap_ratio));
    double info_weight = lc_config_.loop_information_weight *
                         overlap_weight /
                         std::max(best.fitness_score, 1e-4);
    info_weight = std::clamp(info_weight, 1.0, 1e4);
    loop_edge.information =
        Eigen::Matrix<double, 6, 6>::Identity() * info_weight;
    loop_edge.type = PoseGraphEdge::LOOP;

    pose_graph_.addEdge(loop_edge);
    pending_loops_++;
    stats_.total_loops_accepted++;
  }

  for (size_t i = 0; i < last_results_.size(); ++i) {
    auto& result = last_results_[i];
    if (static_cast<int>(i) != best_result_index && result.verification_passed) {
      result.reject_reason = "not_best_candidate";
    }

    if (!result.accepted) {
      stats_.total_loops_rejected++;
    }

    if (!lc_config_.enable_debug_log) {
      continue;
    }

    if (result.accepted) {
      spdlog::info(
          "[LC] LOOP ACCEPTED: submap {}<->{} fitness={:.4f} inliers={} "
          "ov_src={:.3f} ov_tgt={:.3f} t_delta={:.3f}m r_delta={:.3f}rad score={:.3f}",
          result.query_submap_id, result.match_submap_id, result.fitness_score,
          result.final_inlier_count, result.source_overlap_ratio,
          result.target_overlap_ratio, result.translation_delta,
          result.rotation_delta, result.match_score);
    } else {
      spdlog::info(
          "[LC] LOOP REJECTED: submap {}<->{} reason={} converged={} "
          "fitness={:.4f} init_inliers={} final_inliers={} ov0={:.3f} ov1={:.3f} "
          "heading={:.3f} t_delta={:.3f}m r_delta={:.3f}rad",
          result.query_submap_id, result.match_submap_id, result.reject_reason,
          result.converged, result.fitness_score, result.initial_inlier_count,
          result.final_inlier_count, result.initial_source_overlap_ratio,
          result.source_overlap_ratio, result.heading_delta_rad,
          result.translation_delta, result.rotation_delta);
    }
  }

  if (csv_open_ && !last_results_.empty()) {
    writeDebugCsv(last_results_);
  }

  return last_results_;
}

std::vector<size_t> LoopClosureManager::findCandidates(
    const SubMap& query) const {
  std::vector<std::pair<size_t, double>> candidates_with_dist;
  const Eigen::Vector3d query_centroid =
      computeCurrentBestSubmapCentroidLocked(query);

  for (size_t i = 0; i < submaps_.size(); ++i) {
    const auto& sm = submaps_[i];

    // Skip self and recent submaps
    if (sm.id >= query.id) continue;
    int submap_gap = static_cast<int>(query.id) - static_cast<int>(sm.id);
    if (submap_gap < lc_config_.loop_min_submap_gap) continue;

    // Check keyframe gap
    int kf_gap = static_cast<int>(query.first_keyframe_id) -
                 static_cast<int>(sm.last_keyframe_id);
    if (kf_gap < lc_config_.loop_min_keyframe_gap) continue;

    // Spatial proximity check in current-best graph state
    const Eigen::Vector3d candidate_centroid =
        computeCurrentBestSubmapCentroidLocked(sm);
    const double dist = (query_centroid - candidate_centroid).norm();
    if (dist <= lc_config_.loop_search_radius) {
      candidates_with_dist.emplace_back(i, dist);
    }
  }

  // Sort by distance (closest first)
  std::sort(candidates_with_dist.begin(), candidates_with_dist.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });

  // Limit candidates
  std::vector<size_t> result;
  int limit = std::min(static_cast<int>(candidates_with_dist.size()),
                       lc_config_.max_candidates_per_query);
  for (int i = 0; i < limit; ++i) {
    result.push_back(candidates_with_dist[i].first);
  }

  return result;
}

GicpMatchResult LoopClosureManager::verifyWithGicp(
    const SubMap& query, const SubMap& candidate) const {
  auto t_start = std::chrono::steady_clock::now();

  const SubMap query_current = buildCurrentBestMatchingSubmapLocked(query);
  const SubMap candidate_current = buildCurrentBestMatchingSubmapLocked(candidate);

  GicpMatchResult result;
  result.query_submap_id = query.id;
  result.match_submap_id = candidate.id;
  result.query_keyframe_id = query.first_keyframe_id;
  result.match_keyframe_id = candidate.first_keyframe_id;
  result.spatial_distance =
      (query_current.centroid - candidate_current.centroid).norm();
  result.keyframe_gap =
      static_cast<int>(query.first_keyframe_id) -
      static_cast<int>(candidate.last_keyframe_id);

  result.query_centroid_x = query_current.centroid.x();
  result.query_centroid_y = query_current.centroid.y();
  result.query_centroid_z = query_current.centroid.z();
  result.match_centroid_x = candidate_current.centroid.x();
  result.match_centroid_y = candidate_current.centroid.y();
  result.match_centroid_z = candidate_current.centroid.z();
  result.heading_delta_rad = yawDeltaAbs(query_current.reference_pose.q_wb,
                                         candidate_current.reference_pose.q_wb);

  auto query_cloud = lc_config_.project_to_2d
                         ? filterAndProject(query_current.cloud)
                         : query_current.cloud;
  auto cand_cloud = lc_config_.project_to_2d
                        ? filterAndProject(candidate_current.cloud)
                        : candidate_current.cloud;

  result.query_points = query_cloud->size();
  result.match_points = cand_cloud->size();

  if (query_cloud->size() < 30 || cand_cloud->size() < 30) {
    result.reject_reason = "insufficient_points";
    auto t_end = std::chrono::steady_clock::now();
    result.total_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return result;
  }

  if (result.heading_delta_rad > lc_config_.heading_max_delta) {
    result.reject_reason = "heading_gate";
    auto t_end = std::chrono::steady_clock::now();
    result.total_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return result;
  }

  const Eigen::Matrix4d T_query_ref =
      se3::toTransform(query_current.reference_pose.q_wb, query_current.reference_pose.p_wb);
  const Eigen::Matrix4d T_cand_ref =
      se3::toTransform(candidate_current.reference_pose.q_wb, candidate_current.reference_pose.p_wb);
  const Eigen::Matrix4d T_init = se3::inverseSE3(T_cand_ref) * T_query_ref;

  const OverlapStats initial_stats =
      computeOverlapStats(query_cloud, cand_cloud, T_init, lc_config_.overlap_max_distance);
  result.initial_inlier_count = initial_stats.inlier_count;
  result.initial_source_overlap_ratio = initial_stats.source_overlap_ratio;
  result.initial_target_overlap_ratio = initial_stats.target_overlap_ratio;

  if (result.initial_inlier_count < lc_config_.min_initial_inliers) {
    result.reject_reason = "initial_inliers_low";
    auto t_end = std::chrono::steady_clock::now();
    result.total_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return result;
  }

  if (result.initial_source_overlap_ratio < lc_config_.min_initial_overlap_ratio) {
    result.reject_reason = "initial_overlap_low";
    auto t_end = std::chrono::steady_clock::now();
    result.total_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return result;
  }

  if (result.initial_target_overlap_ratio < lc_config_.min_initial_target_overlap_ratio) {
    result.reject_reason = "initial_target_overlap_low";
    auto t_end = std::chrono::steady_clock::now();
    result.total_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return result;
  }

  auto t_gicp_start = std::chrono::steady_clock::now();

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource(query_cloud);
  gicp.setInputTarget(cand_cloud);
  gicp.setMaximumIterations(lc_config_.gicp_max_iterations);
  gicp.setMaxCorrespondenceDistance(lc_config_.gicp_max_correspondence_distance);
  gicp.setTransformationEpsilon(1e-6);
  gicp.setEuclideanFitnessEpsilon(1e-6);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  gicp.align(aligned, T_init.cast<float>());

  auto t_gicp_end = std::chrono::steady_clock::now();
  result.gicp_time_ms =
      std::chrono::duration<double, std::milli>(t_gicp_end - t_gicp_start).count();

  result.converged = gicp.hasConverged();
  result.fitness_score = gicp.getFitnessScore();
  result.gicp_iterations = lc_config_.gicp_max_iterations;

  if (!result.converged) {
    result.reject_reason = "gicp_not_converged";
    auto t_end = std::chrono::steady_clock::now();
    result.total_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return result;
  }

  result.T_relative = gicp.getFinalTransformation().cast<double>();
  const Eigen::Matrix4d T_correction = se3::inverseSE3(T_init) * result.T_relative;
  const Eigen::Matrix<double, 6, 1> xi_correction = se3::LogSE3(T_correction);
  result.rotation_delta = xi_correction.head<3>().norm();
  result.translation_delta = xi_correction.tail<3>().norm();

  const OverlapStats final_stats =
      computeOverlapStats(query_cloud, cand_cloud, result.T_relative, lc_config_.overlap_max_distance);
  result.final_inlier_count = final_stats.inlier_count;
  result.num_correspondences = final_stats.inlier_count;
  result.source_overlap_ratio = final_stats.source_overlap_ratio;
  result.target_overlap_ratio = final_stats.target_overlap_ratio;
  result.median_inlier_distance = final_stats.median_distance;
  result.p90_inlier_distance = final_stats.p90_distance;

  result.match_score =
      result.source_overlap_ratio * std::max(0.05, result.target_overlap_ratio) /
      std::max(result.fitness_score, 1e-4);

  if (result.fitness_score >= lc_config_.gicp_fitness_threshold) {
    result.reject_reason = "fitness_high";
  } else if (result.fitness_score <= lc_config_.gicp_min_fitness_threshold) {
    result.reject_reason = "fitness_too_low";
  } else if (result.final_inlier_count < lc_config_.gicp_min_correspondences) {
    result.reject_reason = "final_inliers_low";
  } else if (result.source_overlap_ratio < lc_config_.min_final_overlap_ratio) {
    result.reject_reason = "final_overlap_low";
  } else if (result.target_overlap_ratio < lc_config_.min_final_target_overlap_ratio) {
    result.reject_reason = "target_overlap_low";
  } else if (result.match_score < lc_config_.min_match_score) {
    result.reject_reason = "match_score_low";
  } else if (result.translation_delta >= lc_config_.gicp_max_translation) {
    result.reject_reason = "translation_delta_high";
  } else if (result.rotation_delta >= lc_config_.gicp_max_rotation) {
    result.reject_reason = "rotation_delta_high";
  } else {
    result.verification_passed = true;
    result.reject_reason = "verification_passed";
  }

  auto t_end = std::chrono::steady_clock::now();
  result.total_time_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();

  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LoopClosureManager::filterAndProject(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
  auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filtered->reserve(cloud->size());

  for (const auto& pt : cloud->points) {
    // Height filter in submap reference frame (approx body frame z)
    if (pt.z >= lc_config_.height_filter_min &&
        pt.z <= lc_config_.height_filter_max) {
      pcl::PointXYZ p2d;
      p2d.x = pt.x;
      p2d.y = pt.y;
      p2d.z = 0.0f;  // Project to XY plane
      filtered->push_back(p2d);
    }
  }

  return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LoopClosureManager::voxelFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float voxel_size) const {
  if (!cloud || cloud->empty() || voxel_size <= 0.0f) {
    return cloud;
  }

  auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(voxel_size, voxel_size, voxel_size);
  vg.filter(*filtered);

  return filtered;
}

bool LoopClosureManager::optimizePoseGraph() {
  std::lock_guard<std::mutex> lk(mutex_);

  if (pending_loops_ <= 0) {
    return false;
  }

  const auto before_nodes = pose_graph_.getNodes();
  const auto edges = pose_graph_.getEdges();

  auto t_start = std::chrono::steady_clock::now();
  const bool success = pose_graph_.optimize(lc_config_.pgo_max_iterations);
  auto t_end = std::chrono::steady_clock::now();
  const double opt_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (success) {
    const auto after_nodes = pose_graph_.getNodes();
    const GraphDiagnostics diag =
        computeGraphDiagnostics(before_nodes, after_nodes, edges);
    const auto solver_summary = pose_graph_.lastOptimizationSummary();

    stats_.total_optimizations++;
    stats_.last_optimization_time_ms = opt_ms;
    stats_.last_pgo_cost_before = diag.cost_before;
    stats_.last_pgo_cost_after = diag.cost_after;
    stats_.last_pgo_mean_node_shift_m = diag.mean_node_shift_m;
    stats_.last_pgo_max_node_shift_m = diag.max_node_shift_m;
    stats_.last_pgo_latest_node_shift_m = diag.latest_node_shift_m;
    stats_.last_pgo_mean_loop_residual_before = diag.mean_loop_residual_before;
    stats_.last_pgo_mean_loop_residual_after = diag.mean_loop_residual_after;
    stats_.last_pgo_max_loop_residual_before = diag.max_loop_residual_before;
    stats_.last_pgo_max_loop_residual_after = diag.max_loop_residual_after;
    stats_.T_map_odom = pose_graph_.getMapToOdomCorrection();
    pending_loops_ = 0;

    if (lc_config_.enable_debug_log) {
      spdlog::info(
          "[LC] PGO completed in {:.1f}ms, nodes={} edges={} raw_cost {:.3f}->{:.3f} "
          "node_shift mean/max/latest {:.4f}/{:.4f}/{:.4f}m loop_res mean {:.4f}->{:.4f} max {:.4f}->{:.4f}",
          opt_ms, pose_graph_.numNodes(), pose_graph_.numEdges(),
          diag.cost_before, diag.cost_after, diag.mean_node_shift_m,
          diag.max_node_shift_m, diag.latest_node_shift_m,
          diag.mean_loop_residual_before, diag.mean_loop_residual_after,
          diag.max_loop_residual_before, diag.max_loop_residual_after);
      spdlog::info(
          "[LC] PGO solver: usable={} converged={} iterations={} summary={} ceres_cost {:.6f}->{:.6f}",
          solver_summary.solution_usable, solver_summary.converged,
          solver_summary.iterations, solver_summary.termination_type,
          solver_summary.initial_cost, solver_summary.final_cost);
    }

    // PGO debug CSV output
    pgo_monitor_.writeCsv(stats_.total_optimizations);
    pgo_logger_.logOptimization(before_nodes, after_nodes, edges,
                                solver_summary, stats_.T_map_odom);
  }

  return success;
}

Eigen::Matrix4d LoopClosureManager::getMapToOdomCorrection() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return stats_.T_map_odom;
}

std::vector<GicpMatchResult> LoopClosureManager::lastMatchResults() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return last_results_;
}

LoopClosureStats LoopClosureManager::getStats() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return stats_;
}

PoseGraphSnapshot LoopClosureManager::getPoseGraphSnapshot() const {
  std::lock_guard<std::mutex> lk(mutex_);
  PoseGraphSnapshot snapshot;
  snapshot.nodes = pose_graph_.getNodes();
  snapshot.edges = pose_graph_.getEdges();
  return snapshot;
}

std::vector<KeyframePoseSnapshot> LoopClosureManager::getKeyframePoseSnapshots() const {
  std::lock_guard<std::mutex> lk(mutex_);
  std::vector<KeyframePoseSnapshot> snapshot;
  const auto& keyframes = keyframe_manager_.getAllKeyframes();
  snapshot.reserve(keyframes.size());
  for (const auto& kf : keyframes) {
    KeyframePoseSnapshot item;
    item.id = kf.id;
    if (!pose_graph_.getOptimizedPose(kf.id, item.q, item.p)) {
      item.q = kf.state.q_wb;
      item.p = kf.state.p_wb;
    }
    snapshot.push_back(item);
  }
  return snapshot;
}

VisualSubmapSnapshot LoopClosureManager::buildVisualSubmapSnapshotLocked(
    const SubMap& sm) const {
  VisualSubmapSnapshot snapshot;
  snapshot.id = sm.id;
  snapshot.first_keyframe_id = sm.first_keyframe_id;
  snapshot.last_keyframe_id = sm.last_keyframe_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t kf_id : sm.keyframe_ids) {
    if (kf_id >= keyframe_manager_.size()) {
      continue;
    }
    const auto& kf = keyframe_manager_.getKeyframe(kf_id);
    if (!kf.cloud || kf.cloud->empty()) {
      continue;
    }

    Eigen::Quaterniond q = kf.state.q_wb;
    Eigen::Vector3d p = kf.state.p_wb;
    pose_graph_.getOptimizedPose(kf_id, q, p);

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = p.cast<float>();

    pcl::PointCloud<pcl::PointXYZ> cloud_world;
    pcl::transformPointCloud(*kf.cloud, cloud_world, T);
    *merged += cloud_world;
  }

  if (lc_config_.submap_voxel_size > 0.0f && !merged->empty()) {
    merged = voxelFilter(merged, static_cast<float>(lc_config_.submap_voxel_size));
  }
  *snapshot.world_cloud = *merged;
  return snapshot;
}

std::vector<VisualSubmapSnapshot> LoopClosureManager::getVisualSubmapSnapshots() const {
  std::lock_guard<std::mutex> lk(mutex_);
  std::vector<VisualSubmapSnapshot> snapshots;
  snapshots.reserve(submaps_.size());
  for (const auto& submap : submaps_) {
    snapshots.push_back(buildVisualSubmapSnapshotLocked(submap));
  }
  return snapshots;
}

LoopClosureDebugSnapshot LoopClosureManager::getDebugSnapshot() const {
  std::lock_guard<std::mutex> lk(mutex_);
  LoopClosureDebugSnapshot snapshot;
  snapshot.pose_graph.nodes = pose_graph_.getNodes();
  snapshot.pose_graph.edges = pose_graph_.getEdges();
  const auto& keyframes = keyframe_manager_.getAllKeyframes();
  snapshot.keyframes.reserve(keyframes.size());
  for (const auto& kf : keyframes) {
    KeyframePoseSnapshot item;
    item.id = kf.id;
    if (!pose_graph_.getOptimizedPose(kf.id, item.q, item.p)) {
      item.q = kf.state.q_wb;
      item.p = kf.state.p_wb;
    }
    snapshot.keyframes.push_back(item);
  }
  snapshot.visual_submaps.reserve(submaps_.size());
  for (const auto& submap : submaps_) {
    snapshot.visual_submaps.push_back(buildVisualSubmapSnapshotLocked(submap));
  }
  snapshot.last_results = last_results_;
  return snapshot;
}

size_t LoopClosureManager::numSubmaps() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return submaps_.size();
}

void LoopClosureManager::openDebugCsv(const std::string& path) {
  if (path.empty()) return;

  csv_file_.open(path);
  if (csv_file_.is_open()) {
    csv_open_ = true;
    csv_file_
        << "query_submap,match_submap,"
        << "query_kf,match_kf,"
        << "fitness_score,num_correspondences,initial_inlier_count,final_inlier_count,"
        << "converged,verification_passed,accepted,reject_reason,"
        << "translation_delta_m,rotation_delta_rad,heading_delta_rad,"
        << "spatial_distance_m,keyframe_gap,match_score,"
        << "query_points,match_points,"
        << "initial_source_overlap_ratio,initial_target_overlap_ratio,"
        << "source_overlap_ratio,target_overlap_ratio,"
        << "median_inlier_distance,p90_inlier_distance,"
        << "gicp_time_ms,total_time_ms"
        << ",query_cx,query_cy,query_cz,match_cx,match_cy,match_cz"
        << "\n";
    csv_file_.flush();

    if (lc_config_.enable_debug_log) {
      spdlog::info("[LC] Debug CSV opened: {}", path);
    }
  }
}

void LoopClosureManager::writeDebugCsv(
    const std::vector<GicpMatchResult>& results) {
  if (!csv_open_) return;

  for (const auto& r : results) {
    csv_file_
        << r.query_submap_id << "," << r.match_submap_id << ","
        << r.query_keyframe_id << "," << r.match_keyframe_id << ","
        << r.fitness_score << "," << r.num_correspondences << ","
        << r.initial_inlier_count << "," << r.final_inlier_count << ","
        << (r.converged ? 1 : 0) << "," << (r.verification_passed ? 1 : 0) << ","
        << (r.accepted ? 1 : 0) << "," << r.reject_reason << ","
        << r.translation_delta << "," << r.rotation_delta << "," << r.heading_delta_rad << ","
        << r.spatial_distance << "," << r.keyframe_gap << "," << r.match_score << ","
        << r.query_points << "," << r.match_points << ","
        << r.initial_source_overlap_ratio << "," << r.initial_target_overlap_ratio << ","
        << r.source_overlap_ratio << "," << r.target_overlap_ratio << ","
        << r.median_inlier_distance << "," << r.p90_inlier_distance << ","
        << r.gicp_time_ms << "," << r.total_time_ms
        << "," << r.query_centroid_x << "," << r.query_centroid_y << "," << r.query_centroid_z
        << "," << r.match_centroid_x << "," << r.match_centroid_y << "," << r.match_centroid_z
        << "\n";
  }
  csv_file_.flush();
}

}  // namespace tof_slam
