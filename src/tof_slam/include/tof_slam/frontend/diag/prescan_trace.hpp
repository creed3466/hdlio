// Copyright 2026 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// prescan_trace.hpp — Task #70 U1a fix-hypothesis instrumentation.
//
// Per-LIDAR-frame CSV dump emitted at the entry of preprocess_scan().
// Records the contents of state_history_ (the trajectory snapshot fed
// into motion_undistort) so that two divergent runs can be diffed at
// the frame-granularity to see whether the trajectory length / first /
// last IMU timestamp differs across runs.
//
// Hypothesis (Fix C): if state_history_.size() or its boundary IMU
// timestamps differ at the first divergent frame (frame=842 per Task
// #70 bisection), the U1a race is in IMU coverage at the LIDAR-anchor
// readiness predicate (slam_node.cpp:805-808) — confirming the need to
// align with FAST-LIO2 / iG-LIO sync_packages discipline.
//
// Falsification: if state_history_ is bit-identical at every frame yet
// bp diverges, the race is in IEKF posterior bias propagation, not in
// the IMU buffer.
//
// Off-path invariant: when TOFSLAM_DEBUG_PRESCAN is unset, no state is
// read, no file is opened, no allocation occurs.
//
// Env vars:
//   TOFSLAM_DEBUG_PRESCAN  = "1"           enable trace
//   TOFSLAM_DEBUG_DIR      = <dir>         output base dir (default /tmp)
//   TOFSLAM_PRESCAN_RUN_ID = <id>          logical run id
//                                          (fallback: TOFSLAM_UPSTREAM_RUN_ID,
//                                           then TOFSLAM_RINGBUF_RUN_ID,
//                                           then parse ROS_MASTER_URI port)
//
// CSV schema (single header row):
//   run_id,frame,scan_end_ts,sh_size,sh_first_ts,sh_last_ts,
//   pos_x,pos_y,pos_z,
//   qx,qy,qz,qw,
//   gb_x,gb_y,gb_z,
//   ab_x,ab_y,ab_z,
//   last_imu_time

#ifndef TOF_SLAM_FRONTEND_DIAG_PRESCAN_TRACE_HPP_
#define TOF_SLAM_FRONTEND_DIAG_PRESCAN_TRACE_HPP_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>

namespace tof_slam {
namespace frontend {
namespace diag {

class PrescanTrace {
 public:
  static PrescanTrace& instance() {
    static PrescanTrace inst;
    return inst;
  }

  static bool enabled() noexcept {
    static const bool cached = [] {
      const char* env = std::getenv("TOFSLAM_DEBUG_PRESCAN");
      if (env == nullptr) return false;
      if (env[0] == '\0') return false;
      if (env[0] == '0' && env[1] == '\0') return false;
      return true;
    }();
    return cached;
  }

  // Preference: TOFSLAM_PRESCAN_RUN_ID > TOFSLAM_UPSTREAM_RUN_ID >
  // TOFSLAM_RINGBUF_RUN_ID > ROS_MASTER_URI trailing port > "0".
  static const std::string& instance_run_id() noexcept {
    static const std::string cached = [] {
      auto pick = [](const char* key) -> std::string {
        const char* v = std::getenv(key);
        if (v != nullptr && v[0] != '\0') return std::string(v);
        return std::string();
      };
      std::string id = pick("TOFSLAM_PRESCAN_RUN_ID");
      if (!id.empty()) return id;
      id = pick("TOFSLAM_UPSTREAM_RUN_ID");
      if (!id.empty()) return id;
      id = pick("TOFSLAM_RINGBUF_RUN_ID");
      if (!id.empty()) return id;
      const char* master = std::getenv("ROS_MASTER_URI");
      if (master == nullptr || master[0] == '\0') return std::string("0");
      std::string_view sv(master);
      auto pos = sv.find_last_of(':');
      if (pos == std::string_view::npos || pos + 1 >= sv.size()) {
        return std::string("0");
      }
      std::string port(sv.substr(pos + 1));
      while (!port.empty() && (port.back() == '/' || port.back() == '\n' ||
                                port.back() == '\r')) {
        port.pop_back();
      }
      if (port.empty()) return std::string("0");
      for (char c : port) {
        if (c < '0' || c > '9') return std::string("0");
      }
      return port;
    }();
    return cached;
  }

  // Per-frame emit. No-op when disabled.
  void log(const std::string& run_id,
           int frame,
           double scan_end_ts,
           std::size_t sh_size,
           double sh_first_ts,
           double sh_last_ts,
           float pos_x, float pos_y, float pos_z,
           float qx, float qy, float qz, float qw,
           float gb_x, float gb_y, float gb_z,
           float ab_x, float ab_y, float ab_z,
           double last_imu_time) {
    if (!enabled()) return;
    if (disabled_after_failure_) return;
    std::lock_guard<std::mutex> lock(mu_);
    if (disabled_after_failure_) return;
    if (fp_ == nullptr) {
      open_file_locked();
      if (fp_ == nullptr) {
        disabled_after_failure_ = true;
        return;
      }
      std::fprintf(fp_,
          "run_id,frame,scan_end_ts,sh_size,sh_first_ts,sh_last_ts,"
          "pos_x,pos_y,pos_z,"
          "qx,qy,qz,qw,"
          "gb_x,gb_y,gb_z,"
          "ab_x,ab_y,ab_z,"
          "last_imu_time\n");
    }
    std::fprintf(fp_,
        "%s,%d,%.17g,%zu,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,"
        "%.17g\n",
        run_id.c_str(), frame, scan_end_ts,
        sh_size, sh_first_ts, sh_last_ts,
        static_cast<double>(pos_x), static_cast<double>(pos_y),
        static_cast<double>(pos_z),
        static_cast<double>(qx), static_cast<double>(qy),
        static_cast<double>(qz), static_cast<double>(qw),
        static_cast<double>(gb_x), static_cast<double>(gb_y),
        static_cast<double>(gb_z),
        static_cast<double>(ab_x), static_cast<double>(ab_y),
        static_cast<double>(ab_z),
        last_imu_time);
  }

  ~PrescanTrace() {
    std::lock_guard<std::mutex> lock(mu_);
    if (fp_ != nullptr) {
      std::fflush(fp_);
      std::fclose(fp_);
      fp_ = nullptr;
    }
  }

  PrescanTrace(const PrescanTrace&) = delete;
  PrescanTrace& operator=(const PrescanTrace&) = delete;
  PrescanTrace(PrescanTrace&&) = delete;
  PrescanTrace& operator=(PrescanTrace&&) = delete;

 private:
  PrescanTrace() = default;

  // mu_ must be held.
  void open_file_locked() {
    const char* dir_env = std::getenv("TOFSLAM_DEBUG_DIR");
    std::string path;
    if (dir_env != nullptr && dir_env[0] != '\0') {
      path.assign(dir_env);
      if (path.back() != '/') path.push_back('/');
    } else {
      path.assign("/tmp/");
    }
    path += "prescan_trace_";
    path += instance_run_id();
    path += ".csv";
    fp_ = std::fopen(path.c_str(), "w");
  }

  std::mutex mu_;
  std::FILE* fp_ = nullptr;
  bool disabled_after_failure_ = false;
};

}  // namespace diag
}  // namespace frontend
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_DIAG_PRESCAN_TRACE_HPP_
