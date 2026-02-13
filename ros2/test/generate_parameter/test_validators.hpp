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

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>
#include "fmt/core.h"

namespace test_validators {

// Validates that a string parameter is not empty
inline tl::expected<void, std::string> string_not_empty(
    const rclcpp::Parameter& parameter) {
  std::string param_value = parameter.as_string();
  if (param_value.empty()) {
    return tl::make_unexpected(
        fmt::format("Parameter '{}' cannot be empty", parameter.get_name()));
  }
  return {};
}

// Validates that an integer is positive
inline tl::expected<void, std::string> positive_integer(
    const rclcpp::Parameter& parameter) {
  int param_value = parameter.as_int();
  if (param_value <= 0) {
    return tl::make_unexpected(
        fmt::format("Parameter '{}' must be positive, got {}",
                    parameter.get_name(), param_value));
  }
  return {};
}

// Validates that a double is within a custom range
inline tl::expected<void, std::string> in_range(
    const rclcpp::Parameter& parameter, double min_value, double max_value) {
  double param_value = parameter.as_double();
  if (param_value < min_value || param_value > max_value) {
    return tl::make_unexpected(
        fmt::format("Parameter '{}' value {} is outside range [{}, {}]",
                    parameter.get_name(), param_value, min_value, max_value));
  }
  return {};
}

}  // namespace test_validators
