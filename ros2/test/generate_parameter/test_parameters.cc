// Copyright 2025 Arjuna Ariyaratne
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

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "ros2/test/generate_parameter/cpp_test_parameters.h"

#include "rclcpp/rclcpp.hpp"

TEST(TestGenerateParameters, TestStringLoad) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto param_listener = std::make_shared<test_params::ParamListener>(node);
  auto params = param_listener->get_params();

  EXPECT_EQ(params.string_test, "default_string");
}

TEST(TestGenerateParameters, TestDoubleLoad) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto param_listener = std::make_shared<test_params::ParamListener>(node);
  auto params = param_listener->get_params();

  EXPECT_DOUBLE_EQ(params.double_test, 2.0);
}

TEST(TestGenerateParameters, TestIntLoad) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto param_listener = std::make_shared<test_params::ParamListener>(node);
  auto params = param_listener->get_params();

  EXPECT_EQ(params.int_test, 5L);
}

TEST(TestGenerateParameters, TestBoolLoad) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto param_listener = std::make_shared<test_params::ParamListener>(node);
  auto params = param_listener->get_params();

  EXPECT_EQ(params.bool_test, true);
}

TEST(TestGenerateParameters, TestStringArrayLoad) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto param_listener = std::make_shared<test_params::ParamListener>(node);
  auto params = param_listener->get_params();

  std::vector<std::string> expected = {"string_1", "string_2"};
  EXPECT_EQ(params.string_array_test, expected);
}

TEST(TestGenerateParameters, TestDoubleArrayLoad) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto param_listener = std::make_shared<test_params::ParamListener>(node);
  auto params = param_listener->get_params();

  std::vector<double> expected = {1.0, 2.0, 3.0};
  EXPECT_EQ(params.double_array_test, expected);
}

TEST(TestGenerateParameters, TestIntArrayLoad) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto param_listener = std::make_shared<test_params::ParamListener>(node);
  auto params = param_listener->get_params();

  std::vector<uint64_t> expected = {1, 2, 3, 4};
  EXPECT_EQ(params.int_array_test, expected);
}

TEST(TestGenerateParameters, TestBoolArrayLoad) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto param_listener = std::make_shared<test_params::ParamListener>(node);
  auto params = param_listener->get_params();

  std::vector<bool> expected = {true, false, true};
  EXPECT_EQ(params.bool_array_test, expected);
}

TEST(TestGenerateParameters, TestStringFixed10Load) {
  auto node = rclcpp::Node::make_shared("test_node");
  auto param_listener = std::make_shared<test_params::ParamListener>(node);
  auto params = param_listener->get_params();

  std::string_view fixed_str = params.string_fixed_10_test;
  EXPECT_EQ(fixed_str, "fixed");
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
