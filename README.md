[![CI](https://github.com/mvukov/rules_ros2/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/mvukov/rules_ros2/actions/workflows/main.yml)

# Bazel rules for ROS 2

This repo provides functionality to build and use ROS 2 with Bazel.

## Prerequisites

You will need to install Bazel, see [here](https://docs.bazel.build/versions/master/install.html).
Besides Bazel, you will need a C++ compiler and a Python 3 interpreter.

And no, you don't have to install any ROS 2 packages via `apt`.

The code is developed and tested on Ubuntu 20.04 with gcc 9.x and Python 3.8.

## What works?

Available features:

- Building of C++ and Python nodes.
- C/C++/Python code generation for interfaces (messages, services and actions).
- Defining ROS 2 deployments with `ros2_launch` Bazel macro.
- Defining ROS 2 tests with `ros2_test` Bazel macro.
- Defining ROS 2 plugins with `ros2_plugin` Bazel macro.
- Only CycloneDDS middleware can be used at the moment.
  - Zero copy transport via shared memory backend ([iceoryx](https://github.com/eclipse-iceoryx/iceoryx)) for CycloneDDS.
- Utilities:
  - `ros2_bag` for handling rosbags
  - `ros2_param` for handling parameters
  - `ros2_service` for handling services
  - `ros2_topic` for handling topics

Please take a look at the [examples](examples) folder to get started.

ROS 2 packages are by default locked to versions from [release-humble-20230127](https://github.com/ros2/ros2/releases/tag/release-humble-20230127).

> **NOTE**: Unlike ROS genmsg which refuses to generate code if the deps between
> interface targets are not set correctly, code generation for ROS 2 seems to not
> care about this. If the deps are not correctly set, you'll only see failures
> during compilation of the generated code.

## Alternatives

For alternative approaches, see:

- [`ApexAI/rules_ros`](https://github.com/ApexAI/rules_ros/)
- [`RobotLocomotion/drake-ros/bazel_ros2_rules`](https://github.com/RobotLocomotion/drake-ros/tree/main/bazel_ros2_rules/ros2#alternatives),
  which includes a brief analysis of this and other approaches.
