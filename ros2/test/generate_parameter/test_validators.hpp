#pragma once

#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

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
