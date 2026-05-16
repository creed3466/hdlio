// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// boundary_hash.hpp — PV-3 Build attribution-only stage-boundary instrumentation.
//
// Passive, env-gated observational logger used by the VI04/VI05 attribution
// study. Produces a CSV of per-frame FNV-1a 64-bit hashes of tightly-scoped
// byte views at six pipeline boundaries (B0..B5). Strictly observational:
// nothing in this header performs computation outside the hashing path, and
// all call sites are compiled to a single cached-bool load + branch when the
// `TOFSLAM_DEBUG_BOUNDARY_HASH` env var is unset.
//
// See docs/reports/pvmap_attribution_architect_20260411.md for design spec.
//
// C++17 — `std::span` is C++20, so this header defines a minimal `ByteView`
// shim. FNV-1a remains `constexpr` for unit-test reference values.

#ifndef TOF_SLAM_FRONTEND_DIAG_BOUNDARY_HASH_HPP_
#define TOF_SLAM_FRONTEND_DIAG_BOUNDARY_HASH_HPP_

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>

namespace tof_slam {
namespace frontend {
namespace diag {

// ---------------------------------------------------------------------------
// ByteView — C++17 shim for std::span<const std::byte>.
//
// The attribution study only needs a non-owning view over a contiguous
// byte range, so ByteView is intentionally minimal: a pointer + length.
// ---------------------------------------------------------------------------

struct ByteView {
  const std::byte* data = nullptr;
  std::size_t size = 0;

  constexpr ByteView() noexcept = default;
  constexpr ByteView(const std::byte* d, std::size_t n) noexcept
      : data(d), size(n) {}
};

// ---------------------------------------------------------------------------
// fnv1a_64 — FNV-1a 64-bit hash.
//
// Offset basis: 0xcbf29ce484222325
// Prime       : 0x100000001b3
//
// `constexpr` so unit tests can verify compile-time reference hashes.
// Pure function, no allocations.
// ---------------------------------------------------------------------------

constexpr std::uint64_t kFnv1a64OffsetBasis = 0xcbf29ce484222325ULL;
constexpr std::uint64_t kFnv1a64Prime       = 0x100000001b3ULL;

constexpr std::uint64_t fnv1a_64(ByteView bytes) noexcept {
  std::uint64_t h = kFnv1a64OffsetBasis;
  for (std::size_t i = 0; i < bytes.size; ++i) {
    h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes.data[i]));
    h *= kFnv1a64Prime;
  }
  return h;
}

// Incremental FNV-1a absorb step. Threads the running hash `h` through
// an additional byte range. Used for field-by-field hashing where the
// struct layout contains compiler-inserted padding bytes that must be
// skipped (e.g. `Correspondence` has 3 padding bytes after `bool has_l2`).
constexpr std::uint64_t fnv1a_64_update(std::uint64_t h,
                                         ByteView bytes) noexcept {
  for (std::size_t i = 0; i < bytes.size; ++i) {
    h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes.data[i]));
    h *= kFnv1a64Prime;
  }
  return h;
}

// ---------------------------------------------------------------------------
// BoundaryId — contiguous enum matching CSV column 2.
// ---------------------------------------------------------------------------

enum class BoundaryId : std::uint8_t {
  B0_RawScan      = 0,
  B1_Preprocessed = 1,
  B2_Predicted    = 2,
  B3_Corrs        = 3,
  B4_IekfIter     = 4,
  B5_Output       = 5,
  // --- Task #36 Dark01 PV-4 ordering localizer (env-gated, observational) ---
  // Each hook captures the iteration or insertion signature of one
  // unordered_dense container on the hot path. Used to pinpoint the first
  // boundary at which Class A and Class B Dark01 runs diverge.
  B6_ScanForCorr        = 6,  // Scan vector input to correspondence_finder
  B7_L1CountOrder       = 7,  // l1_count map iteration (corr_finder:59)
  B8_ShareCountOrder    = 8,  // share_count map iteration (corr_finder:283)
  B9_AffectedOrder      = 9,  // affected set iteration (surfel_map:604)
  B10_SurfelDirtyOrder  = 10, // dirty_keys order after sort (surfel_map:641)
};

// ---------------------------------------------------------------------------
// Current-frame index (thread_local) — shared by all 6 boundaries.
//
// LioEstimator::feed_lidar writes this at frame start; every hook reads it.
// Using a thread_local avoids rippling a frame_idx parameter through
// IekfUpdater's free-function signature.
// ---------------------------------------------------------------------------

inline std::uint64_t& current_frame_idx() noexcept {
  static thread_local std::uint64_t idx = 0;
  return idx;
}

inline void set_current_frame_idx(std::uint64_t f) noexcept {
  current_frame_idx() = f;
}

// ---------------------------------------------------------------------------
// BoundaryLogger — Meyers-singleton CSV sink for boundary hashes.
//
// Fast path (disabled): one cached bool load + branch, zero allocations.
// Enabled path: FNV-1a over the given byte view, one mutex-protected
// fprintf to an owned std::FILE*. RAII close in destructor.
//
// CSV schema (no header row):
//   frame_idx,boundary_id,hash64,aux_scalar
// where hash64 is 16-hex-digit `0x%016lx` and aux_scalar is `%.17g`.
// ---------------------------------------------------------------------------

class BoundaryLogger {
 public:
  static BoundaryLogger& instance() {
    static BoundaryLogger inst;
    return inst;
  }

  // Cached env check. Returns true iff TOFSLAM_DEBUG_BOUNDARY_HASH is set
  // to a non-empty, non-"0" value. Single cached read on first call.
  static bool enabled() noexcept {
    static const bool cached = [] {
      const char* env = std::getenv("TOFSLAM_DEBUG_BOUNDARY_HASH");
      if (env == nullptr) return false;
      if (env[0] == '\0') return false;
      if (env[0] == '0' && env[1] == '\0') return false;
      return true;
    }();
    return cached;
  }

  // Configure the CSV filename. Call once from LioEstimator construction.
  // Thread-safe: synchronizes with log() via the internal mutex.
  void set_run_id(std::string_view run_id) {
    std::lock_guard<std::mutex> lock(mu_);
    run_id_.assign(run_id.data(), run_id.size());
    // Close any previously-opened file; it will be re-opened lazily on
    // the next log() call under the new run_id.
    if (fp_ != nullptr) {
      std::fflush(fp_);
      std::fclose(fp_);
      fp_ = nullptr;
    }
  }

  // Append one boundary row. No-op fast path when disabled.
  void log(std::uint64_t frame_idx,
           BoundaryId bid,
           ByteView bytes,
           double aux) {
    if (!enabled()) return;
    const std::uint64_t h = fnv1a_64(bytes);
    log_precomputed(frame_idx, bid, h, aux);
  }

  // Same as log(), but with a caller-precomputed FNV-1a hash. Used by
  // field-by-field hashing paths (e.g. B3) where the raw struct byte
  // range contains padding that must be skipped.
  void log_precomputed(std::uint64_t frame_idx,
                       BoundaryId bid,
                       std::uint64_t hash64,
                       double aux) {
    if (!enabled()) return;
    std::lock_guard<std::mutex> lock(mu_);
    if (fp_ == nullptr) {
      open_file_locked();
      if (fp_ == nullptr) return;  // Could not open; silently drop.
    }
    std::fprintf(fp_,
                 "%llu,%u,0x%016llx,%.17g\n",
                 static_cast<unsigned long long>(frame_idx),
                 static_cast<unsigned>(bid),
                 static_cast<unsigned long long>(hash64),
                 aux);
  }

  // Test helper: force re-read of the env var cache. Not for production use.
  // Only used by the unit test after (un)setenv() to flip the cached bool.
  // Safe because the cache is function-local static.
  // NB: we intentionally do NOT expose a way to clear `enabled()`'s cache
  // because it is static-local; tests must control env before first call.

  ~BoundaryLogger() {
    std::lock_guard<std::mutex> lock(mu_);
    if (fp_ != nullptr) {
      std::fflush(fp_);
      std::fclose(fp_);
      fp_ = nullptr;
    }
  }

  // Non-copyable, non-movable.
  BoundaryLogger(const BoundaryLogger&) = delete;
  BoundaryLogger& operator=(const BoundaryLogger&) = delete;
  BoundaryLogger(BoundaryLogger&&) = delete;
  BoundaryLogger& operator=(BoundaryLogger&&) = delete;

 private:
  BoundaryLogger() = default;

  // mu_ must be held.
  void open_file_locked() {
    const char* dump_dir_env = std::getenv("DUMP_DIR");
    std::string path;
    if (dump_dir_env != nullptr && dump_dir_env[0] != '\0') {
      path.assign(dump_dir_env);
      if (path.back() != '/') path.push_back('/');
    }
    path += "boundary_";
    if (run_id_.empty()) {
      const char* rid_env = std::getenv("TOFSLAM_BOUNDARY_RUN_ID");
      if (rid_env != nullptr && rid_env[0] != '\0') {
        run_id_.assign(rid_env);
      } else {
        run_id_.assign("default");
      }
    }
    path += run_id_;
    path += ".csv";
    fp_ = std::fopen(path.c_str(), "w");
  }

  std::mutex mu_;
  std::FILE* fp_ = nullptr;
  std::string run_id_;
};

// ---------------------------------------------------------------------------
// Convenience helpers used at hook sites.
// ---------------------------------------------------------------------------

// Build a ByteView from any pointer + byte-count pair.
inline ByteView make_byte_view(const void* ptr, std::size_t n_bytes) noexcept {
  return ByteView{static_cast<const std::byte*>(ptr), n_bytes};
}

}  // namespace diag
}  // namespace frontend
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_DIAG_BOUNDARY_HASH_HPP_
