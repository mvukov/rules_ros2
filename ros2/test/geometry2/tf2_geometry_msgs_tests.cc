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
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {

geometry_msgs::msg::TransformStamped MakeTranslationTransform(
    const std::string& parent_frame, const std::string& child_frame, double tx,
    double ty, double tz) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent_frame;
  t.child_frame_id = child_frame;
  t.transform.translation.x = tx;
  t.transform.translation.y = ty;
  t.transform.translation.z = tz;
  t.transform.rotation.w = 1.0;
  return t;
}

TEST(TestTf2GeometryMsgs, WhenPointTransformed_EnsureFrameIdUpdated) {
  geometry_msgs::msg::PointStamped point_in, point_out;
  point_in.header.frame_id = "child";

  const auto transform = MakeTranslationTransform("parent", "child", 0, 0, 0);
  tf2::doTransform(point_in, point_out, transform);

  EXPECT_EQ(point_out.header.frame_id, "parent");
}

TEST(TestTf2GeometryMsgs, WhenPoseTransformed_EnsureFrameIdUpdated) {
  geometry_msgs::msg::PoseStamped pose_in, pose_out;
  pose_in.header.frame_id = "child";
  pose_in.pose.orientation.w = 1.0;

  const auto transform = MakeTranslationTransform("parent", "child", 0, 0, 0);
  tf2::doTransform(pose_in, pose_out, transform);

  EXPECT_EQ(pose_out.header.frame_id, "parent");
}

TEST(TestTf2GeometryMsgs, WhenIdentityTransformApplied_EnsureValuesUnchanged) {
  geometry_msgs::msg::PointStamped point_in, point_out;
  point_in.header.frame_id = "child";
  point_in.point.x = 1.0;
  point_in.point.y = 2.0;
  point_in.point.z = 3.0;

  const auto transform = MakeTranslationTransform("parent", "child", 0, 0, 0);
  tf2::doTransform(point_in, point_out, transform);

  EXPECT_DOUBLE_EQ(point_out.point.x, 1.0);
  EXPECT_DOUBLE_EQ(point_out.point.y, 2.0);
  EXPECT_DOUBLE_EQ(point_out.point.z, 3.0);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
