// Copyright 2023 Foxglove Technologies Inc
// SPDX-License-Identifier: MIT
// Adapted from https://github.com/foxglove/ros-foxglove-bridge at 0.5.3.
// In the current setting Bazel rules don't support ROS 2 components, therefore,
// the node is directly instantiated here.
#include "foxglove_bridge/ros2_foxglove_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  size_t numThreads = 0;
  {
    // Temporary dummy node to get num_threads param.
    auto dummyNode = std::make_shared<rclcpp::Node>("dummy");
    auto numThreadsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    numThreadsDescription.name = "num_threads";
    numThreadsDescription.type =
        rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    numThreadsDescription.description =
        "The number of threads to use for the ROS node executor. 0 means one "
        "thread per CPU core.";
    numThreadsDescription.read_only = true;
    numThreadsDescription.additional_constraints =
        "Must be a non-negative integer";
    numThreadsDescription.integer_range.resize(1);
    numThreadsDescription.integer_range[0].from_value = 0;
    numThreadsDescription.integer_range[0].to_value = INT32_MAX;
    numThreadsDescription.integer_range[0].step = 1;
    constexpr int DEFAULT_NUM_THREADS = 0;
    dummyNode->declare_parameter(numThreadsDescription.name,
                                 DEFAULT_NUM_THREADS, numThreadsDescription);
    numThreads = static_cast<size_t>(
        dummyNode->get_parameter(numThreadsDescription.name).as_int());
  }

  auto executor = rclcpp::executors::MultiThreadedExecutor(
      rclcpp::ExecutorOptions{}, numThreads);

  auto node = std::make_shared<foxglove_bridge::FoxgloveBridge>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
