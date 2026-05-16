// Copyright 2026 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// upstream_trace.hpp — Task #70 U1 upstream bisection.
//
// Env-gated CSV trace of every per-point (body, world, state) triplet
// at map-update time, immediately after transform_to_world() and before
// update_map(). Disabled path = single cached-bool branch per frame.
//
// Mirrors ringbuf_trace.hpp header-only pattern. Off-path invariant:
// when TOFSLAM_DEBUG_UPSTREAM is unset, no state is read, no file is
// opened, no allocation occurs.
//
// Phase 3 bisection goal: given the ringbuf_trace divergence at
// (frame=843, voxel=(59,-40,-3), n_before=96) with cause-a (different p),
// split the race into:
//   U1a  bp differs between classes  → preprocess / IMU undistortion race
//   U1b  bp identical, state differs → IEKF inner iteration race
//   U1c  bp & state identical, wp differs → transform_to_world bug
//        (IEEE-754 impossible under single-thread scalar Eigen)
//
// Env vars:
//   TOFSLAM_DEBUG_UPSTREAM = "1"          enable trace
//   TOFSLAM_DEBUG_DIR      = <dir>        output base dir (default /tmp)
//   TOFSLAM_UPSTREAM_RUN_ID = <id>        logical run id
//                                         (fallback: TOFSLAM_RINGBUF_RUN_ID,
//                                          then parse ROS_MASTER_URI port)
//
// CSV schema (single header row):
//   run_id,frame,idx_in_frame,
//   bx,by,bz,
//   wx,wy,wz,
//   sx,sy,sz,
//   qx,qy,qz,qw

#ifndef TOF_SLAM_FRONTEND_DIAG_UPSTREAM_TRACE_HPP_
#define TOF_SLAM_FRONTEND_DIAG_UPSTREAM_TRACE_HPP_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>

namespace tof_slam {
namespace frontend {
namespace diag {

class UpstreamTrace {
 public:
  static UpstreamTrace& instance() {
    static UpstreamTrace inst;
    return inst;
  }

  static bool enabled() noexcept {
    static const bool cached = [] {
      const char* env = std::getenv("TOFSLAM_DEBUG_UPSTREAM");
      if (env == nullptr) return false;
      if (env[0] == '\0') return false;
      if (env[0] == '0' && env[1] == '\0') return false;
      return true;
    }();
    return cached;
  }

  // Preference: TOFSLAM_UPSTREAM_RUN_ID > TOFSLAM_RINGBUF_RUN_ID >
  // ROS_MASTER_URI trailing port > "0".
  static const std::string& instance_run_id() noexcept {
    static const std::string cached = [] {
      auto pick = [](const char* key) -> std::string {
        const char* v = std::getenv(key);
        if (v != nullptr && v[0] != '\0') return std::string(v);
        return std::string();
      };
      std::string id = pick("TOFSLAM_UPSTREAM_RUN_ID");
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

  // Per-point emit. No-op when disabled.
  void log(const std::string& run_id,
           int frame,
           int idx_in_frame,
           float bx, float by, float bz,
           float wx, float wy, float wz,
           float sx, float sy, float sz,
           float qx, float qy, float qz, float qw) {
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
          "run_id,frame,idx_in_frame,"
          "bx,by,bz,"
          "wx,wy,wz,"
          "sx,sy,sz,"
          "qx,qy,qz,qw\n");
    }
    std::fprintf(fp_,
        "%s,%d,%d,"
        "%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,%.17g\n",
        run_id.c_str(), frame, idx_in_frame,
        static_cast<double>(bx), static_cast<double>(by), static_cast<double>(bz),
        static_cast<double>(wx), static_cast<double>(wy), static_cast<double>(wz),
        static_cast<double>(sx), static_cast<double>(sy), static_cast<double>(sz),
        static_cast<double>(qx), static_cast<double>(qy), static_cast<double>(qz),
        static_cast<double>(qw));
  }

  ~UpstreamTrace() {
    std::lock_guard<std::mutex> lock(mu_);
    if (fp_ != nullptr) {
      std::fflush(fp_);
      std::fclose(fp_);
      fp_ = nullptr;
    }
  }

  UpstreamTrace(const UpstreamTrace&) = delete;
  UpstreamTrace& operator=(const UpstreamTrace&) = delete;
  UpstreamTrace(UpstreamTrace&&) = delete;
  UpstreamTrace& operator=(UpstreamTrace&&) = delete;

 private:
  UpstreamTrace() = default;

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
    path += "upstream_trace_";
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

#endif  // TOF_SLAM_FRONTEND_DIAG_UPSTREAM_TRACE_HPP_
