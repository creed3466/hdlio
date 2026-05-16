// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// boundary_hash_test.cpp — PV-3 Build unit test for the FNV-1a hash and
// BoundaryLogger env gate. See
// docs/reports/pvmap_attribution_architect_20260411.md §10 (R1 mitigation).

#include "tof_slam/frontend/diag/boundary_hash.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>

namespace tof_slam {
namespace frontend {
namespace diag {
namespace {

// ---------------------------------------------------------------------------
// Helper: convert a C-string literal to a ByteView (excluding NUL).
// Not constexpr — reinterpret_cast is disallowed in constant expressions.
// ---------------------------------------------------------------------------
ByteView make_view_from_cstr(const char* s, std::size_t n) {
  return ByteView{reinterpret_cast<const std::byte*>(s), n};
}

// ---------------------------------------------------------------------------
// Test 1 — Reference hash on "foo".
//
// Python reference (confirmed offline):
//   def fnv1a_64(bs):
//       h = 0xcbf29ce484222325
//       for b in bs:
//           h ^= b
//           h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
//       return h
//   fnv1a_64(b'foo')         -> 0xdcb27518fed9d577
//   fnv1a_64(b'hello world') -> 0x779a65e7023cd2e7
//   fnv1a_64(b'')            -> 0xcbf29ce484222325
// ---------------------------------------------------------------------------

TEST(BoundaryHashTest, Fnv1aFooReferenceHash) {
  const char kInput[] = "foo";  // 3 bytes
  const auto view = make_view_from_cstr(kInput, 3);
  EXPECT_EQ(fnv1a_64(view), 0xdcb27518fed9d577ULL);
}

TEST(BoundaryHashTest, Fnv1aHelloWorldReferenceHash) {
  const char kInput[] = "hello world";  // 11 bytes
  const auto view = make_view_from_cstr(kInput, 11);
  EXPECT_EQ(fnv1a_64(view), 0x779a65e7023cd2e7ULL);
}

TEST(BoundaryHashTest, Fnv1aEmptyIsOffsetBasis) {
  // Empty span must return the FNV-1a offset basis (no iterations).
  const ByteView empty{};
  EXPECT_EQ(fnv1a_64(empty), 0xcbf29ce484222325ULL);
  // Sanity: our constant matches the named constant.
  EXPECT_EQ(kFnv1a64OffsetBasis, 0xcbf29ce484222325ULL);
}

// ---------------------------------------------------------------------------
// Test 2 — BoundaryLogger::enabled() returns false with env unset.
//
// `BoundaryLogger::enabled()` caches on first call, so this test relies
// on the env being unset BEFORE any other code path in the process
// touches the singleton. In the unit-test binary this is the first (and
// only) caller, so the cache is established here and remains false.
// ---------------------------------------------------------------------------

TEST(BoundaryHashTest, LoggerDisabledWithoutEnvVar) {
  ::unsetenv("TOFSLAM_DEBUG_BOUNDARY_HASH");
  EXPECT_FALSE(BoundaryLogger::enabled());

  // Fast-path log() call: must be a no-op when disabled — it should
  // not crash, allocate, or attempt to open a file.
  const char kData[] = "no-op";
  BoundaryLogger::instance().log(
      /*frame_idx=*/0,
      BoundaryId::B0_RawScan,
      ByteView{reinterpret_cast<const std::byte*>(kData), 5},
      /*aux=*/0.0);
}

// ---------------------------------------------------------------------------
// Test 3 — current_frame_idx() round trip.
// ---------------------------------------------------------------------------

TEST(BoundaryHashTest, CurrentFrameIdxRoundTrip) {
  set_current_frame_idx(0);
  EXPECT_EQ(current_frame_idx(), 0ULL);
  set_current_frame_idx(42);
  EXPECT_EQ(current_frame_idx(), 42ULL);
  set_current_frame_idx(0);
}

}  // namespace
}  // namespace diag
}  // namespace frontend
}  // namespace tof_slam

// gtest main — catkin_add_gtest does not always link against gtest_main
// for this project's ROS1 build, so provide one directly.
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
