// Copyright 2026 Milan Vukov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "Eigen/Geometry"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace {

geometry_msgs::msg::TransformStamped MakeTransformStamped(double tx, double ty,
                                                          double tz,
                                                          double qw) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "parent";
  t.child_frame_id = "child";
  t.transform.translation.x = tx;
  t.transform.translation.y = ty;
  t.transform.translation.z = tz;
  t.transform.rotation.w = qw;
  return t;
}

TEST(TestTf2Eigen,
     WhenIdentityTransformConverted_EnsureEigenIsometry3dIsIdentity) {
  const auto transform = MakeTransformStamped(0.0, 0.0, 0.0, 1.0);
  const Eigen::Isometry3d isometry = tf2::transformToEigen(transform);
  EXPECT_TRUE(isometry.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(TestTf2Eigen, WhenEigenIsometry3dConverted_EnsureTranslationMatches) {
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  const auto transform = tf2::eigenToTransform(isometry);
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, 2.0);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 3.0);
}

TEST(TestTf2Eigen, WhenPointTransformed_EnsureValueCorrect) {
  geometry_msgs::msg::TransformStamped transform =
      MakeTransformStamped(1.0, 2.0, 3.0, 1.0);

  const Eigen::Vector3d point_in(0.0, 0.0, 0.0);
  Eigen::Vector3d point_out;
  tf2::doTransform(point_in, point_out, transform);

  EXPECT_DOUBLE_EQ(point_out.x(), 1.0);
  EXPECT_DOUBLE_EQ(point_out.y(), 2.0);
  EXPECT_DOUBLE_EQ(point_out.z(), 3.0);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
