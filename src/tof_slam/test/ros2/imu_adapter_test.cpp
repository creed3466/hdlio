// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// imu_adapter_test.cpp — Unit tests for ImuAdapter.

#include "tof_slam/ros2/imu_adapter.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace tof_slam {
namespace ros_adapter {
namespace {

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------

/// Build a minimal ROS Imu message with the given fields.
sensor_msgs::msg::Imu::SharedPtr make_imu_msg(
    double timestamp,
    const Eigen::Vector3f& gyro,
    const Eigen::Vector3f& accel) {
  auto msg = std::make_shared<sensor_msgs::msg::Imu>();

  // Encode timestamp into sec / nanosec.
  msg->header.stamp.sec     = static_cast<int32_t>(timestamp);
  msg->header.stamp.nanosec = static_cast<uint32_t>(
      (timestamp - msg->header.stamp.sec) * 1e9);

  msg->angular_velocity.x = gyro.x();
  msg->angular_velocity.y = gyro.y();
  msg->angular_velocity.z = gyro.z();

  msg->linear_acceleration.x = accel.x();
  msg->linear_acceleration.y = accel.y();
  msg->linear_acceleration.z = accel.z();

  return msg;
}

/// Push n identical messages into the adapter. Returns the last result.
std::optional<core::ImuMeasurement> feed(ImuAdapter& adapter,
                                         int n,
                                         const Eigen::Vector3f& accel,
                                         double start_ts = 0.0,
                                         double dt = 0.01) {
  std::optional<core::ImuMeasurement> last;
  for (int i = 0; i < n; ++i) {
    last = adapter.process(
        make_imu_msg(start_ts + i * dt,
                     Eigen::Vector3f::Zero(),
                     accel));
  }
  return last;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(ImuAdapterTest, DefaultNotInitialized) {
  ImuAdapter adapter;
  EXPECT_FALSE(adapter.initialized());
  EXPECT_FALSE(adapter.init_attempted());
}

TEST(ImuAdapterTest, BufferingReturnsNullopt) {
  ImuAdapter adapter;
  // Feed 99 samples — all should return nullopt.
  for (int i = 0; i < 99; ++i) {
    auto result = adapter.process(
        make_imu_msg(i * 0.01, Eigen::Vector3f::Zero(),
                     Eigen::Vector3f(0.f, 0.f, 9.81f)));
    EXPECT_FALSE(result.has_value()) << "sample " << i << " should be buffered";
  }
  EXPECT_FALSE(adapter.initialized());
}

TEST(ImuAdapterTest, InitializesAtThreshold) {
  ImuAdapter adapter;
  // Stationary sensor pointing up (m/s^2).
  Eigen::Vector3f accel(0.f, 0.f, 9.81f);
  feed(adapter, 100, accel);
  EXPECT_TRUE(adapter.init_attempted());
  EXPECT_TRUE(adapter.initialized());
}

TEST(ImuAdapterTest, ReturnsDataAfterInit) {
  ImuAdapter adapter;
  feed(adapter, 100, Eigen::Vector3f(0.f, 0.f, 9.81f));
  ASSERT_TRUE(adapter.initialized());

  const double ts = 1.0;
  const Eigen::Vector3f gyro(0.1f, 0.2f, 0.3f);
  const Eigen::Vector3f accel(0.f, 0.f, 9.81f);
  auto result = adapter.process(make_imu_msg(ts, gyro, accel));

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->timestamp, ts, 1e-9);
}

TEST(ImuAdapterTest, ConvertTimestamp) {
  ImuAdapter adapter;
  feed(adapter, 100, Eigen::Vector3f(0.f, 0.f, 9.81f));
  ASSERT_TRUE(adapter.initialized());

  // 1 second + 500 ms → 1.5 s
  const double expected_ts = 1.5;
  auto msg = make_imu_msg(expected_ts, Eigen::Vector3f::Zero(),
                          Eigen::Vector3f(0.f, 0.f, 9.81f));
  auto result = adapter.process(msg);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->timestamp, expected_ts, 1e-6);
}

TEST(ImuAdapterTest, ConvertGyroAccel) {
  ImuAdapter adapter;
  feed(adapter, 100, Eigen::Vector3f(0.f, 0.f, 9.81f));
  ASSERT_TRUE(adapter.initialized());

  const Eigen::Vector3f gyro(0.1f, -0.2f, 0.3f);
  const Eigen::Vector3f accel(1.f, 2.f, -3.f);
  auto result = adapter.process(make_imu_msg(2.0, gyro, accel));

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->gyro.x(), gyro.x(), 1e-5f);
  EXPECT_NEAR(result->gyro.y(), gyro.y(), 1e-5f);
  EXPECT_NEAR(result->gyro.z(), gyro.z(), 1e-5f);
  EXPECT_NEAR(result->accel.x(), accel.x(), 1e-5f);
  EXPECT_NEAR(result->accel.y(), accel.y(), 1e-5f);
  EXPECT_NEAR(result->accel.z(), accel.z(), 1e-5f);
}

TEST(ImuAdapterTest, GravityInitResult) {
  // Stationary sensor with acc = [0, 0, 9.81] → already in m/s^2
  // imu_acc_scale should be ≈ 1.0.
  ImuAdapter adapter;
  feed(adapter, 100, Eigen::Vector3f(0.f, 0.f, 9.81f));

  ASSERT_TRUE(adapter.initialized());
  EXPECT_TRUE(adapter.init_result().success);
  EXPECT_NEAR(adapter.init_result().imu_acc_scale, 1.0f, 0.1f);
}

TEST(ImuAdapterTest, GUnitDetection) {
  // MID-360 style: acc ≈ [0, 0, 1.0] (g-units)
  // imu_acc_scale should be ≈ 9.81.
  ImuAdapter adapter;
  feed(adapter, 100, Eigen::Vector3f(0.f, 0.f, 1.0f));

  ASSERT_TRUE(adapter.initialized());
  EXPECT_TRUE(adapter.init_result().success);
  EXPECT_NEAR(adapter.init_result().imu_acc_scale, 9.81f, 0.5f);
}

TEST(ImuAdapterTest, Reset) {
  ImuAdapter adapter;
  feed(adapter, 100, Eigen::Vector3f(0.f, 0.f, 9.81f));
  ASSERT_TRUE(adapter.initialized());

  adapter.reset();
  EXPECT_FALSE(adapter.initialized());
  EXPECT_FALSE(adapter.init_attempted());

  // After reset the adapter should buffer again.
  auto result = adapter.process(
      make_imu_msg(0.0, Eigen::Vector3f::Zero(),
                   Eigen::Vector3f(0.f, 0.f, 9.81f)));
  EXPECT_FALSE(result.has_value());
}

TEST(ImuAdapterTest, CustomSampleCount) {
  // Use a smaller buffer so init triggers after 50 samples.
  ImuAdapter::Config cfg;
  cfg.init_sample_count = 50;
  ImuAdapter adapter(cfg);

  // 49 samples → still buffering.
  feed(adapter, 49, Eigen::Vector3f(0.f, 0.f, 9.81f));
  EXPECT_FALSE(adapter.initialized());

  // 50th sample triggers init.
  feed(adapter, 1, Eigen::Vector3f(0.f, 0.f, 9.81f), 49 * 0.01);
  EXPECT_TRUE(adapter.initialized());
}

}  // namespace
}  // namespace ros_adapter
}  // namespace tof_slam
