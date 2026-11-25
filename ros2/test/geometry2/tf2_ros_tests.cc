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
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace {

geometry_msgs::msg::TransformStamped MakeIdentityTransform(
    const std::string& parent_frame, const std::string& child_frame) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent_frame;
  t.child_frame_id = child_frame;
  t.transform.rotation.w = 1.0;
  return t;
}

TEST(TestTf2RosBuffer, WhenBufferCreated_EnsureInitiallyEmpty) {
  auto clock = std::make_shared<rclcpp::Clock>();
  tf2_ros::Buffer buffer(clock);
  EXPECT_TRUE(buffer.getAllFrameNames().empty());
}

TEST(TestTf2RosBuffer, WhenTransformSetDirectly_EnsureLookupSucceeds) {
  auto clock = std::make_shared<rclcpp::Clock>();
  tf2_ros::Buffer buffer(clock);
  buffer.setTransform(MakeIdentityTransform("base", "child"), "test",
                      /*is_static=*/false);

  const auto result =
      buffer.lookupTransform("base", "child", tf2::TimePointZero);
  EXPECT_EQ(result.header.frame_id, "base");
  EXPECT_EQ(result.child_frame_id, "child");
}

TEST(TestTf2RosTransformBroadcaster, WhenCreated_EnsureNoException) {
  auto node = rclcpp::Node::make_shared("tf2_ros_broadcaster_test_node");
  EXPECT_NO_THROW(tf2_ros::TransformBroadcaster broadcaster(node));
}

TEST(TestTf2RosStaticTransformBroadcaster,
     WhenStaticTransformSent_EnsureBufferContainsTransform) {
  auto node = rclcpp::Node::make_shared("tf2_ros_static_broadcaster_test_node");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener listener(buffer, node, /*spin_thread=*/false);
  tf2_ros::StaticTransformBroadcaster broadcaster(node);

  broadcaster.sendTransform(MakeIdentityTransform("world", "sensor"));

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    if (buffer.canTransform("world", "sensor", tf2::TimePointZero)) {
      break;
    }
  }

  EXPECT_TRUE(buffer.canTransform("world", "sensor", tf2::TimePointZero));
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  if (!rclcpp::shutdown() || result) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
