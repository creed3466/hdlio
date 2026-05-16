// Copyright 2026 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// ringbuf_trace.hpp — Task #70 Proposal A instrumentation.
//
// Env-gated CSV trace of every L0 ring-buffer insertion. Strictly
// observational: disabled path is a single cached-bool load + branch,
// identical to `BoundaryLogger` pattern.
//
// Architect contract: docs/reports/task70_proposal_a_architect_20260414.md
// Codex review:      docs/reports/task70_proposal_a_codex_review_20260414.md
// Upstream research: docs/reports/task70_c3_research_analysis_20260414.md
//
// Drift note vs architect §3.1: implemented header-only (no separate
// ringbuf_trace.cpp, no CMakeLists change) — mirrors proven
// boundary_hash.hpp pattern. Codex-mandated M1 (c7) "no fmt::format"
// is satisfied via std::fprintf("%.17g"), which is already the
// boundary_hash.hpp idiom. Off-path invariant (§4) unchanged.
//
// Env vars:
//   TOFSLAM_DEBUG_RINGBUF = "1"            enables trace
//   TOFSLAM_DEBUG_DIR      = <dir>         output base dir (default /tmp)
//   TOFSLAM_RINGBUF_RUN_ID = <id>          logical run id
//                                          (fallback: parse from ROS_MASTER_URI trailing port)
//
// CSV schema (single header row on first write):
//   run_id,frame,voxel_key_x,voxel_key_y,voxel_key_z,
//   n_before_add,ring_head_before,
//   px,py,pz,
//   pre_sum_x,pre_sum_y,pre_sum_z,
//   post_sum_x,post_sum_y,post_sum_z,
//   centroid_x,centroid_y,centroid_z

#ifndef TOF_SLAM_FRONTEND_DIAG_RINGBUF_TRACE_HPP_
#define TOF_SLAM_FRONTEND_DIAG_RINGBUF_TRACE_HPP_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>

namespace tof_slam {
namespace frontend {
namespace diag {

class RingbufTrace {
 public:
  static RingbufTrace& instance() {
    static RingbufTrace inst;
    return inst;
  }

  // Cached env-var read. One branch on the hot path when disabled.
  static bool enabled() noexcept {
    static const bool cached = [] {
      const char* env = std::getenv("TOFSLAM_DEBUG_RINGBUF");
      if (env == nullptr) return false;
      if (env[0] == '\0') return false;
      if (env[0] == '0' && env[1] == '\0') return false;
      return true;
    }();
    return cached;
  }

  // Cached run_id (env or ROS_MASTER_URI trailing port or "0").
  static const std::string& instance_run_id() noexcept {
    static const std::string cached = [] {
      const char* explicit_id = std::getenv("TOFSLAM_RINGBUF_RUN_ID");
      if (explicit_id != nullptr && explicit_id[0] != '\0') {
        return std::string(explicit_id);
      }
      const char* master = std::getenv("ROS_MASTER_URI");
      if (master == nullptr || master[0] == '\0') return std::string("0");
      std::string_view sv(master);
      auto pos = sv.find_last_of(':');
      if (pos == std::string_view::npos || pos + 1 >= sv.size()) {
        return std::string("0");
      }
      // Strip any trailing '/' from e.g. "http://localhost:11311/"
      std::string port(sv.substr(pos + 1));
      while (!port.empty() && (port.back() == '/' || port.back() == '\n' ||
                                port.back() == '\r')) {
        port.pop_back();
      }
      if (port.empty()) return std::string("0");
      // Validate digits-only; fall back to "0" on parse failure.
      for (char c : port) {
        if (c < '0' || c > '9') return std::string("0");
      }
      return port;
    }();
    return cached;
  }

  // One row per ring-buffer insertion. No-op when disabled.
  void log(const std::string& run_id,
           int frame,
           int kx, int ky, int kz,
           int n_before_add,
           int ring_head_before,
           float px, float py, float pz,
           float pre_sx, float pre_sy, float pre_sz,
           float post_sx, float post_sy, float post_sz,
           float cx, float cy, float cz) {
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
      // Header on first write.
      std::fprintf(fp_,
          "run_id,frame,voxel_key_x,voxel_key_y,voxel_key_z,"
          "n_before_add,ring_head_before,"
          "px,py,pz,"
          "pre_sum_x,pre_sum_y,pre_sum_z,"
          "post_sum_x,post_sum_y,post_sum_z,"
          "centroid_x,centroid_y,centroid_z\n");
    }
    // IEEE-754 round-trip precision via %.17g (boundary_hash.hpp idiom).
    std::fprintf(fp_,
        "%s,%d,%d,%d,%d,%d,%d,"
        "%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g,"
        "%.17g,%.17g,%.17g\n",
        run_id.c_str(), frame, kx, ky, kz,
        n_before_add, ring_head_before,
        static_cast<double>(px), static_cast<double>(py), static_cast<double>(pz),
        static_cast<double>(pre_sx), static_cast<double>(pre_sy), static_cast<double>(pre_sz),
        static_cast<double>(post_sx), static_cast<double>(post_sy), static_cast<double>(post_sz),
        static_cast<double>(cx), static_cast<double>(cy), static_cast<double>(cz));
  }

  ~RingbufTrace() {
    std::lock_guard<std::mutex> lock(mu_);
    if (fp_ != nullptr) {
      std::fflush(fp_);
      std::fclose(fp_);
      fp_ = nullptr;
    }
  }

  RingbufTrace(const RingbufTrace&) = delete;
  RingbufTrace& operator=(const RingbufTrace&) = delete;
  RingbufTrace(RingbufTrace&&) = delete;
  RingbufTrace& operator=(RingbufTrace&&) = delete;

 private:
  RingbufTrace() = default;

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
    path += "ringbuf_trace_";
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

#endif  // TOF_SLAM_FRONTEND_DIAG_RINGBUF_TRACE_HPP_
