// Copyright 2023 Laurenz Altenmueller
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

#include <cstdlib>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  // If neither $ROS_HOME nor $ROS_LOG_DIR are set to a valid directory,
  // rclcpp:init() will fail with these errors:
  // clang-format off
  // 'rcutils_expand_user failed, at external/ros2_rcl_logging/rcl_logging_interface/src/logging_dir.c:81'             // NOLINT
  // 'Failed to get logging directory, at external/ros2_rcl_logging/rcl_logging_spdlog/src/rcl_logging_spdlog.cpp:83'  // NOLINT
  // clang-format on
  rclcpp::init(argc, argv);
  return rclcpp::shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
