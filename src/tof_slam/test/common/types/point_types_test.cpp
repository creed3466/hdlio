// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/common/types/point_types.hpp"

#include <gtest/gtest.h>

namespace tof_slam {
namespace core {
namespace {

constexpr float kTol = 1e-5f;

// ===========================================================================
// Point3D
// ===========================================================================

TEST(Point3DTest, DefaultIsZero) {
  Point3D p;
  EXPECT_FLOAT_EQ(p.x, 0.0f);
  EXPECT_FLOAT_EQ(p.y, 0.0f);
  EXPECT_FLOAT_EQ(p.z, 0.0f);
}

TEST(Point3DTest, Arithmetic) {
  Point3D a(1.0f, 2.0f, 3.0f);
  Point3D b(4.0f, 5.0f, 6.0f);
  Point3D sum = a + b;
  EXPECT_FLOAT_EQ(sum.x, 5.0f);
  EXPECT_FLOAT_EQ(sum.y, 7.0f);
  EXPECT_FLOAT_EQ(sum.z, 9.0f);

  Point3D diff = b - a;
  EXPECT_FLOAT_EQ(diff.x, 3.0f);

  Point3D scaled = a * 2.0f;
  EXPECT_FLOAT_EQ(scaled.x, 2.0f);
}

TEST(Point3DTest, Norm) {
  Point3D p(3.0f, 4.0f, 0.0f);
  EXPECT_NEAR(p.norm(), 5.0f, kTol);
  EXPECT_NEAR(p.squared_norm(), 25.0f, kTol);
}

TEST(Point3DTest, EigenRoundTrip) {
  Point3D p(1.5f, -2.3f, 4.1f);
  Eigen::Vector3f v = p.to_eigen();
  Point3D q = Point3D::from_eigen(v);
  EXPECT_FLOAT_EQ(q.x, p.x);
  EXPECT_FLOAT_EQ(q.y, p.y);
  EXPECT_FLOAT_EQ(q.z, p.z);
}

// ===========================================================================
// PointCloud
// ===========================================================================

TEST(PointCloudTest, PushBackAndSize) {
  PointCloud cloud;
  EXPECT_TRUE(cloud.empty());
  cloud.push_back(Point3D(1, 2, 3));
  cloud.push_back(Point3D(4, 5, 6));
  EXPECT_EQ(cloud.size(), 2u);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0f);
}

TEST(PointCloudTest, Clear) {
  PointCloud cloud;
  cloud.push_back(Point3D(1, 2, 3));
  cloud.clear();
  EXPECT_TRUE(cloud.empty());
}

TEST(PointCloudTest, TransformIdentity) {
  PointCloud cloud;
  cloud.push_back(Point3D(1, 2, 3));
  cloud.transform(Se3::Identity());
  EXPECT_NEAR(cloud[0].x, 1.0f, kTol);
  EXPECT_NEAR(cloud[0].y, 2.0f, kTol);
  EXPECT_NEAR(cloud[0].z, 3.0f, kTol);
}

TEST(PointCloudTest, TransformTranslation) {
  PointCloud cloud;
  cloud.push_back(Point3D(0, 0, 0));

  Se3 T(So3::Identity(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  cloud.transform(T);
  EXPECT_NEAR(cloud[0].x, 1.0f, kTol);
  EXPECT_NEAR(cloud[0].y, 2.0f, kTol);
  EXPECT_NEAR(cloud[0].z, 3.0f, kTol);
}

TEST(PointCloudTest, TransformedCopyPreservesOriginal) {
  PointCloud cloud;
  cloud.push_back(Point3D(1, 0, 0));

  Se3 T(So3::Identity(), Eigen::Vector3f(10.0f, 0.0f, 0.0f));
  PointCloud copy = cloud.transformed_copy(T);
  EXPECT_NEAR(cloud[0].x, 1.0f, kTol);   // Original unchanged
  EXPECT_NEAR(copy[0].x, 11.0f, kTol);   // Copy transformed
}

TEST(PointCloudTest, TransformedCopyPreservesIntensity) {
  PointCloud cloud;
  cloud.push_back(Point3D(0, 0, 0, 42.0f, 0.5f));

  Se3 T(So3::Identity(), Eigen::Vector3f(1, 0, 0));
  PointCloud copy = cloud.transformed_copy(T);
  EXPECT_FLOAT_EQ(copy[0].intensity, 42.0f);
  EXPECT_FLOAT_EQ(copy[0].offset_time, 0.5f);
}

TEST(PointCloudTest, BoundingBox) {
  PointCloud cloud;
  cloud.push_back(Point3D(-1, -2, -3));
  cloud.push_back(Point3D(4, 5, 6));
  cloud.push_back(Point3D(0, 0, 0));

  Eigen::Vector3f min_pt, max_pt;
  cloud.bounding_box(min_pt, max_pt);
  EXPECT_FLOAT_EQ(min_pt.x(), -1.0f);
  EXPECT_FLOAT_EQ(max_pt.z(), 6.0f);
}

TEST(PointCloudTest, BoundingBoxEmpty) {
  PointCloud cloud;
  Eigen::Vector3f min_pt, max_pt;
  cloud.bounding_box(min_pt, max_pt);
  EXPECT_EQ(min_pt, Eigen::Vector3f::Zero());
}

TEST(PointCloudTest, Centroid) {
  PointCloud cloud;
  cloud.push_back(Point3D(0, 0, 0));
  cloud.push_back(Point3D(2, 4, 6));
  Eigen::Vector3f c = cloud.centroid();
  EXPECT_NEAR(c.x(), 1.0f, kTol);
  EXPECT_NEAR(c.y(), 2.0f, kTol);
  EXPECT_NEAR(c.z(), 3.0f, kTol);
}

TEST(PointCloudTest, RangeBasedFor) {
  PointCloud cloud;
  cloud.push_back(Point3D(1, 0, 0));
  cloud.push_back(Point3D(0, 1, 0));
  float sum_x = 0.0f;
  for (const auto& p : cloud) {
    sum_x += p.x;
  }
  EXPECT_NEAR(sum_x, 1.0f, kTol);
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
