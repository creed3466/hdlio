// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// point_cloud_adapter_test.cpp — Unit tests for PointCloudAdapter.

#include "tof_slam/ros2/point_cloud_adapter.hpp"

#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace tof_slam {
namespace ros_adapter {
namespace {

// ===========================================================================
// Test-cloud builder
// ===========================================================================

/// Append a named field of the given datatype and return the new offset.
static uint32_t add_field(
    sensor_msgs::msg::PointCloud2& msg,
    const std::string& name,
    uint8_t datatype,
    uint32_t offset) {
  sensor_msgs::msg::PointField f;
  f.name     = name;
  f.offset   = offset;
  f.datatype = datatype;
  f.count    = 1;
  msg.fields.push_back(f);

  // Return offset past this field.
  switch (datatype) {
    case sensor_msgs::msg::PointField::FLOAT32: return offset + 4;
    case sensor_msgs::msg::PointField::FLOAT64: return offset + 8;
    case sensor_msgs::msg::PointField::UINT32:  return offset + 4;
    default:                                    return offset + 4;
  }
}

/// Build a PointCloud2 message with specified fields and point data.
///
/// @param points       XYZ data for each point (may be NaN to test skipping).
/// @param intensities  Per-point intensity; ignored when has_intensity=false.
/// @param has_intensity Whether to include the "intensity" field.
/// @param time_field   "" (none), "timestamp" (double abs s), "offset_time" (uint32 ns).
/// @param time_values  Per-point time values (double seconds or uint32 nanoseconds).
/// @param header_stamp_sec  Seconds component of header stamp.
/// @param header_stamp_nsec Nanoseconds component of header stamp.
sensor_msgs::msg::PointCloud2::SharedPtr make_test_cloud(
    const std::vector<std::array<float, 3>>& points,
    const std::vector<float>& intensities,
    bool has_intensity,
    const std::string& time_field,
    const std::vector<double>& time_values,
    uint32_t header_stamp_sec  = 1000,
    uint32_t header_stamp_nsec = 0) {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec    = header_stamp_sec;
  msg->header.stamp.nanosec = header_stamp_nsec;
  msg->is_bigendian = false;

  // Build field layout.
  uint32_t off = 0;
  off = add_field(*msg, "x", sensor_msgs::msg::PointField::FLOAT32, off);
  off = add_field(*msg, "y", sensor_msgs::msg::PointField::FLOAT32, off);
  off = add_field(*msg, "z", sensor_msgs::msg::PointField::FLOAT32, off);
  if (has_intensity) {
    off = add_field(*msg, "intensity", sensor_msgs::msg::PointField::FLOAT32, off);
  }
  if (time_field == "timestamp") {
    off = add_field(*msg, "timestamp", sensor_msgs::msg::PointField::FLOAT64, off);
  } else if (time_field == "offset_time") {
    off = add_field(*msg, "offset_time", sensor_msgs::msg::PointField::UINT32, off);
  }

  msg->point_step = off;
  msg->width      = static_cast<uint32_t>(points.size());
  msg->height     = 1;
  msg->row_step   = msg->point_step * msg->width;
  msg->data.resize(static_cast<size_t>(msg->row_step), 0);

  // Fill point data.
  for (size_t i = 0; i < points.size(); ++i) {
    uint8_t* row = msg->data.data() + i * msg->point_step;

    // x/y/z at offsets 0/4/8.
    std::memcpy(row + 0, &points[i][0], 4);
    std::memcpy(row + 4, &points[i][1], 4);
    std::memcpy(row + 8, &points[i][2], 4);

    uint32_t field_off = 12;

    if (has_intensity) {
      float intens = (i < intensities.size()) ? intensities[i] : 0.0f;
      std::memcpy(row + field_off, &intens, 4);
      field_off += 4;
    }

    if (time_field == "timestamp" && i < time_values.size()) {
      double ts = time_values[i];
      std::memcpy(row + field_off, &ts, 8);
    } else if (time_field == "offset_time" && i < time_values.size()) {
      auto ot_ns = static_cast<uint32_t>(time_values[i]);
      std::memcpy(row + field_off, &ot_ns, 4);
    }
  }

  return msg;
}

// ===========================================================================
// Tests
// ===========================================================================

TEST(PointCloudAdapterTest, BasicXYZ) {
  // 10 points, no intensity, no time → all converted, intensity=0, offset_time=0.
  std::vector<std::array<float, 3>> pts(10);
  for (int i = 0; i < 10; ++i) {
    pts[i] = {static_cast<float>(i), static_cast<float>(i * 2), 1.0f};
  }

  auto msg = make_test_cloud(pts, {}, /*has_intensity=*/false, "", {});
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg);

  ASSERT_EQ(cloud.size(), 10u);
  for (size_t i = 0; i < 10u; ++i) {
    EXPECT_FLOAT_EQ(cloud[i].x, static_cast<float>(i));
    EXPECT_FLOAT_EQ(cloud[i].y, static_cast<float>(i * 2));
    EXPECT_FLOAT_EQ(cloud[i].z, 1.0f);
    EXPECT_FLOAT_EQ(cloud[i].intensity,   0.0f);
    EXPECT_FLOAT_EQ(cloud[i].offset_time, 0.0f);
  }
}

TEST(PointCloudAdapterTest, WithIntensity) {
  // Points with intensity field → intensity values preserved.
  std::vector<std::array<float, 3>> pts = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
  std::vector<float> intens             = {100.0f, 200.0f};

  auto msg = make_test_cloud(pts, intens, /*has_intensity=*/true, "", {});
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg);

  ASSERT_EQ(cloud.size(), 2u);
  EXPECT_FLOAT_EQ(cloud[0].intensity, 100.0f);
  EXPECT_FLOAT_EQ(cloud[1].intensity, 200.0f);
  // Coordinates are correct.
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0f);
  EXPECT_FLOAT_EQ(cloud[1].z, 6.0f);
}

TEST(PointCloudAdapterTest, WithOffsetTimeNanos) {
  // Points with uint32 "offset_time" in nanoseconds → converted to float seconds.
  std::vector<std::array<float, 3>> pts = {{1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}};
  // 0 ns and 100,000,000 ns = 0.1 s.
  std::vector<double> time_ns = {0.0, 100000000.0};

  auto msg = make_test_cloud(pts, {}, /*has_intensity=*/false,
                             "offset_time", time_ns);
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg);

  ASSERT_EQ(cloud.size(), 2u);
  EXPECT_NEAR(cloud[0].offset_time, 0.0f,   1e-6f);
  EXPECT_NEAR(cloud[1].offset_time, 0.1f,   1e-6f);
}

TEST(PointCloudAdapterTest, WithTimestampDouble) {
  // Points with double "timestamp" field → offset relative to scan time.
  //
  // Scan header stamp = 1000.0 s.
  // Point timestamps = 1000.0 s and 1000.05 s.
  // Expected offset_time: 0.0 s and 0.05 s.
  constexpr uint32_t sec  = 1000;
  constexpr uint32_t nsec = 0;
  constexpr double scan_ts = 1000.0;

  std::vector<std::array<float, 3>> pts = {{1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}};
  std::vector<double> abs_ts = {scan_ts, scan_ts + 0.05};

  auto msg = make_test_cloud(pts, {}, /*has_intensity=*/false,
                             "timestamp", abs_ts, sec, nsec);
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg);

  ASSERT_EQ(cloud.size(), 2u);
  EXPECT_NEAR(cloud[0].offset_time, 0.00f, 1e-6f);
  EXPECT_NEAR(cloud[1].offset_time, 0.05f, 1e-6f);
}

TEST(PointCloudAdapterTest, WithTimestampDoubleExplicitScanTs) {
  // Use the two-argument overload with an explicit scan_timestamp.
  constexpr double scan_ts = 500.0;

  std::vector<std::array<float, 3>> pts = {{0.0f, 0.0f, 0.0f}};
  std::vector<double> abs_ts = {scan_ts + 0.01};

  auto msg = make_test_cloud(pts, {}, /*has_intensity=*/false,
                             "timestamp", abs_ts,
                             /*header_stamp_sec=*/0, /*header_stamp_nsec=*/0);
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg, scan_ts);

  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_NEAR(cloud[0].offset_time, 0.01f, 1e-6f);
}

TEST(PointCloudAdapterTest, EmptyCloud) {
  // Empty message → empty PointCloud.
  auto msg = make_test_cloud({}, {}, /*has_intensity=*/false, "", {});
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg);

  EXPECT_TRUE(cloud.empty());
}

TEST(PointCloudAdapterTest, SkipsNaNPoints) {
  // Points with NaN x → skipped; valid points are kept.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::vector<std::array<float, 3>> pts = {
      {1.0f,  0.0f, 0.0f},   // valid
      {nan,   0.0f, 0.0f},   // NaN x → skip
      {0.0f,  nan,  0.0f},   // NaN y → skip
      {0.0f,  0.0f, nan},    // NaN z → skip
      {2.0f,  0.0f, 0.0f},   // valid
  };

  auto msg = make_test_cloud(pts, {}, /*has_intensity=*/false, "", {});
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg);

  ASSERT_EQ(cloud.size(), 2u);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0f);
  EXPECT_FLOAT_EQ(cloud[1].x, 2.0f);
}

TEST(PointCloudAdapterTest, SkipsInfPoints) {
  // Points with Inf x → skipped.
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<std::array<float, 3>> pts = {
      {inf,  0.0f, 0.0f},   // +Inf x → skip
      {-inf, 0.0f, 0.0f},   // -Inf x → skip
      {1.0f, 0.0f, 0.0f},   // valid
  };

  auto msg = make_test_cloud(pts, {}, /*has_intensity=*/false, "", {});
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg);

  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0f);
}

TEST(PointCloudAdapterTest, PointCountPreserved) {
  // N valid points → exactly N in output.
  constexpr size_t N = 50;
  std::vector<std::array<float, 3>> pts(N);
  for (size_t i = 0; i < N; ++i) {
    pts[i] = {static_cast<float>(i), 0.0f, 0.0f};
  }

  auto msg = make_test_cloud(pts, {}, /*has_intensity=*/false, "", {});
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg);

  EXPECT_EQ(cloud.size(), N);
}

TEST(PointCloudAdapterTest, NullMessageReturnsEmpty) {
  PointCloudAdapter adapter;
  sensor_msgs::msg::PointCloud2::SharedPtr null_msg;
  auto cloud = adapter.convert(null_msg);
  EXPECT_TRUE(cloud.empty());
}

TEST(PointCloudAdapterTest, IntensityAndOffsetTimeCombined) {
  // Intensity + offset_time together are both correctly populated.
  std::vector<std::array<float, 3>> pts = {{3.0f, 4.0f, 5.0f}};
  std::vector<float> intens             = {77.0f};
  std::vector<double> time_ns           = {50000000.0};  // 0.05 s

  auto msg = make_test_cloud(pts, intens, /*has_intensity=*/true,
                             "offset_time", time_ns);
  PointCloudAdapter adapter;
  auto cloud = adapter.convert(msg);

  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud[0].intensity, 77.0f);
  EXPECT_NEAR(cloud[0].offset_time, 0.05f, 1e-6f);
  EXPECT_FLOAT_EQ(cloud[0].x, 3.0f);
}

}  // namespace
}  // namespace ros_adapter
}  // namespace tof_slam
