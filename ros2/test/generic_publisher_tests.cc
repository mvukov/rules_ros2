// Copyright 2023 Milan Vukov
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
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

// Inspired by an example in https://github.com/ros2/rclcpp/issues/2146
// For this to work, a rclcpp patch from
// https://github.com/mvukov/rules_ros2/pull/117 is required.
TEST(TestGenericPublisher,
     WhenPublisherDeletedInThread_EnsureCleanNodeCleanup) {
  auto node = rclcpp::Node::make_shared("pub_node");
  auto publisher =
      node->create_generic_publisher("pub_topic", "std_msgs/String", 10);

  auto reset_publisher = std::async(std::launch::async, [&node, &publisher]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    publisher.reset();

    // New: Create a timer that gets immediately out of scope (should never
    // fire) to trigger cleanup of the publisher.
    node->create_wall_timer(std::chrono::seconds(1), []() {});
  });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_until_future_complete(reset_publisher);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  if (!rclcpp::shutdown() || result) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
