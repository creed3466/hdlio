// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// csv_logger.cpp — CSV trajectory and diagnostics file writer implementation.

#include "tof_slam/ros/csv_logger.hpp"

#include <iomanip>

namespace tof_slam {

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------

CsvLogger::~CsvLogger() {
  close();
}

// ---------------------------------------------------------------------------
// open — open trajectory CSV and optionally diagnostics CSV
// ---------------------------------------------------------------------------

void CsvLogger::open(const std::string& trajectory_csv_path,
                     const std::string& dump_path,
                     bool enable_diag) {
  {
    std::lock_guard<std::mutex> lk(csv_mutex_);
    csv_file_.open(trajectory_csv_path, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
      csv_file_ << "seq,t_sec,"
                   "tx,ty,tz,qx,qy,qz,qw,"
                   "odom_tx,odom_ty,odom_tz,odom_qx,odom_qy,odom_qz,odom_qw\n";
    }
    // logged by caller
  }

  if (enable_diag) {
    // Derive stem from trajectory_csv_path
    std::string stem = trajectory_csv_path;
    const auto slash = stem.rfind('/');
    if (slash != std::string::npos) stem = stem.substr(slash + 1);
    const auto dot = stem.rfind('.');
    if (dot != std::string::npos) stem = stem.substr(0, dot);

    const std::string diag_path = dump_path + "/" + stem + "_diag.csv";
    std::lock_guard<std::mutex> lk(diag_csv_mutex_);
    diag_csv_.open(diag_path, std::ios::out | std::ios::trunc);
    if (diag_csv_.is_open()) {
      diag_csv_ <<
        "frame_id,t_sec,"
        "corr_count,corr_planarity_avg,corr_res_rms,lidar_rms_prior,lidar_rms_post,trust_score,"
        "trust_window_mean,accept_window_hits,"
        "iekf_converged,iekf_iters,iekf_pos_delta,iekf_rot_delta,"
        "low_corr,low_corr_consecutive,skip_map_update,accepted_lidar_update,blended_lidar_update,blend_alpha,map_commit_allowed,"
        "objective_improved,wheel_safe,ground_safe,rms_safe,window_consistent,"
        "reject_reason_code,accept_reason_code,commit_reason_code,"
        "vel_mag,wheel_scale,gyro_bias_z,"
        "P_rot_x,P_rot_y,P_rot_z,"
        "P_pos_x,P_pos_y,P_pos_z,"
        "P_vel_x,P_vel_y,P_scale,"
        "wv_res_vx,wv_res_omega,wv_info_vx,wv_info_omega,"
        "wheel_cost_prior,wheel_cost_post,ground_cost_prior,ground_cost_post,"
        "pose_delta_norm,yaw_delta_abs,obs_min_eig,obs_cond_ratio,"
        "map_l0,map_l1\n";
    }
    // logged by caller
  }
}

// ---------------------------------------------------------------------------
// close — close all open file handles
// ---------------------------------------------------------------------------

void CsvLogger::close() {
  {
    std::lock_guard<std::mutex> lk(csv_mutex_);
    if (csv_file_.is_open()) csv_file_.close();
  }
  {
    std::lock_guard<std::mutex> lk(diag_csv_mutex_);
    if (diag_csv_.is_open()) diag_csv_.close();
  }
  {
    std::lock_guard<std::mutex> lk(usage_csv_mutex_);
    if (usage_csv_.is_open()) usage_csv_.close();
  }
}

// ---------------------------------------------------------------------------
// write_trajectory — LWO trajectory row with relative odom
// ---------------------------------------------------------------------------

void CsvLogger::write_trajectory(const Eigen::Vector3f& position,
                                  const Eigen::Quaternionf& q,
                                  double timestamp,
                                  const Eigen::Vector3f& rel_odom_pos,
                                  const Eigen::Quaternionf& rel_odom_q) {
  std::lock_guard<std::mutex> lk(csv_mutex_);
  if (!csv_file_.is_open()) return;
  csv_file_ << csv_seq_++ << ","
            << std::fixed << std::setprecision(9)
            << timestamp        << ","
            << position.x()     << "," << position.y() << ","
            << position.z()     << ","
            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
            << rel_odom_pos.x() << "," << rel_odom_pos.y() << ","
            << rel_odom_pos.z() << ","
            << rel_odom_q.x()   << "," << rel_odom_q.y() << ","
            << rel_odom_q.z()   << "," << rel_odom_q.w()
            << "\n";
  csv_file_.flush();  // Ensure all data is written to disk (prevents loss on SIGKILL)
}

// ---------------------------------------------------------------------------
// write_trajectory_lio — LIO trajectory row (no odom, identity relative pose)
// ---------------------------------------------------------------------------

void CsvLogger::write_trajectory_lio(const Eigen::Vector3f& position,
                                      const Eigen::Quaternionf& q,
                                      double timestamp) {
  std::lock_guard<std::mutex> lk(csv_mutex_);
  if (!csv_file_.is_open()) return;
  csv_file_ << csv_seq_++ << ","
            << std::fixed << std::setprecision(9)
            << timestamp        << ","
            << position.x()     << "," << position.y() << ","
            << position.z()     << ","
            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
            << "0,0,0,0,0,0,1\n";
}

// ---------------------------------------------------------------------------
// write_diagnostics — LWO per-frame diagnostics row
// ---------------------------------------------------------------------------

void CsvLogger::write_diagnostics(const lwo::LwoEstimator::FrameDiagnostics& d) {
  std::lock_guard<std::mutex> lk(diag_csv_mutex_);
  if (!diag_csv_.is_open()) return;
  diag_csv_
    << std::fixed << std::setprecision(9)
    << d.frame_id                  << ","
    << d.timestamp                 << ","
    << d.corr_count                << ","
    << std::setprecision(6)
    << d.corr_planarity_avg        << ","
    << d.corr_res_rms              << ","
    << d.lidar_rms_prior           << ","
    << d.lidar_rms_post            << ","
    << d.trust_score               << ","
    << d.trust_window_mean         << ","
    << d.accept_window_hits        << ","
    << (d.iekf_converged ? 1 : 0)  << ","
    << d.iekf_iters                << ","
    << d.iekf_pos_delta            << ","
    << d.iekf_rot_delta            << ","
    << (d.low_corr ? 1 : 0)        << ","
    << d.low_corr_consecutive      << ","
    << (d.skip_map_update ? 1 : 0) << ","
    << (d.accepted_lidar_update ? 1 : 0) << ","
    << (d.blended_lidar_update ? 1 : 0) << ","
    << d.blend_alpha               << ","
    << (d.map_commit_allowed ? 1 : 0) << ","
    << (d.objective_improved ? 1 : 0) << ","
    << (d.wheel_safe ? 1 : 0)      << ","
    << (d.ground_safe ? 1 : 0)     << ","
    << (d.rms_safe ? 1 : 0)        << ","
    << (d.window_consistent ? 1 : 0) << ","
    << d.reject_reason_code        << ","
    << d.accept_reason_code        << ","
    << d.commit_reason_code        << ","
    << d.vel_mag                   << ","
    << d.wheel_scale               << ","
    << d.gyro_bias_z               << ","
    << d.P_rot_x                   << ","
    << d.P_rot_y                   << ","
    << d.P_rot_z                   << ","
    << d.P_pos_x                   << ","
    << d.P_pos_y                   << ","
    << d.P_pos_z                   << ","
    << d.P_vel_x                   << ","
    << d.P_vel_y                   << ","
    << d.P_scale                   << ","
    << d.wv_residual_vx            << ","
    << d.wv_residual_omega         << ","
    << d.wv_info_vx                << ","
    << d.wv_info_omega             << ","
    << d.wheel_cost_prior          << ","
    << d.wheel_cost_post           << ","
    << d.ground_cost_prior         << ","
    << d.ground_cost_post          << ","
    << d.pose_delta_norm           << ","
    << d.yaw_delta_abs             << ","
    << d.obs_min_eig               << ","
    << d.obs_cond_ratio            << ","
    << d.map_l0                    << ","
    << d.map_l1
    << "\n";
  diag_csv_.flush();
}

// ---------------------------------------------------------------------------
// open_usage — open usage CSV
// ---------------------------------------------------------------------------

void CsvLogger::open_usage(const std::string& dump_path,
                            const std::string& stem) {
  const std::string path = dump_path + "/" + stem + "_usage.csv";
  std::lock_guard<std::mutex> lk(usage_csv_mutex_);
  usage_csv_.open(path, std::ios::out | std::ios::trunc);
  if (usage_csv_.is_open()) {
    usage_csv_ <<
      "frame_id,t_sec,"
      "total_ms,preprocess_ms,corr_find_ms,anchor_corr_ms,"
      "iekf_ms,anchor_yaw_ms,map_update_ms,"
      "iekf_jacobian_ms,iekf_build_ms,iekf_solve_ms,wheel_ms,"
      "cpu_percent,rss_mb,queue_depth,"
      "outer_iters,total_inner_iters,corr_count,scan_points\n";
  }
}

// ---------------------------------------------------------------------------
// write_usage — LWO per-frame resource usage row
// ---------------------------------------------------------------------------

void CsvLogger::write_usage(const lwo::LwoEstimator::FrameUsage& u) {
  std::lock_guard<std::mutex> lk(usage_csv_mutex_);
  if (!usage_csv_.is_open()) return;
  usage_csv_
    << std::fixed << std::setprecision(9)
    << u.frame_id              << ","
    << u.timestamp             << ","
    << std::setprecision(3)
    << u.total_ms              << "," << u.preprocess_ms      << ","
    << u.corr_find_ms          << "," << u.anchor_corr_ms     << ","
    << u.iekf_ms               << "," << u.anchor_yaw_ms      << ","
    << u.map_update_ms         << ","
    << u.iekf_jacobian_ms      << "," << u.iekf_build_ms      << ","
    << u.iekf_solve_ms         << "," << u.wheel_ms           << ","
    << std::setprecision(1)
    << u.cpu_percent           << ","
    << std::setprecision(2)
    << u.rss_mb                << ","
    << u.queue_depth           << ","
    << u.outer_iters           << "," << u.total_inner_iters  << ","
    << u.corr_count            << "," << u.scan_points
    << "\n";
  usage_csv_.flush();
}

}  // namespace tof_slam
