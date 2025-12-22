#!/bin/bash

set -o errexit -o nounset -o pipefail

if [ -f "ros2/test/pluginlib/py_loader_tests_binary" ]; then
  ros2/test/pluginlib/py_loader_tests_binary
elif [ -f "rules_ros2/ros2/test/pluginlib/py_loader_tests_binary" ]; then
  rules_ros2/ros2/test/pluginlib/py_loader_tests_binary
elif [ -f "../rules_ros2+/ros2/test/pluginlib/py_loader_tests_binary" ]; then
  ../rules_ros2+/ros2/test/pluginlib/py_loader_tests_binary
elif [ -f "../com_github_mvukov_rules_ros2/ros2/test/pluginlib/py_loader_tests_binary" ]; then
  ../com_github_mvukov_rules_ros2/ros2/test/pluginlib/py_loader_tests_binary
else
  echo "Binary not found"
  find . -name py_loader_tests_binary
  exit 1
fi
