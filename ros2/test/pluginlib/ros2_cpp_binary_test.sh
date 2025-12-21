#!/bin/bash

set -o errexit -o nounset -o pipefail

if [ -f "ros2/test/pluginlib/plugin_tests_binary" ]; then
  ros2/test/pluginlib/plugin_tests_binary
elif [ -f "rules_ros2/ros2/test/pluginlib/plugin_tests_binary" ]; then
  rules_ros2/ros2/test/pluginlib/plugin_tests_binary
else
  echo "Binary not found"
  find . -name plugin_tests_binary
  exit 1
fi
