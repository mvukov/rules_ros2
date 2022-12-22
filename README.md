[![CI](https://github.com/mvukov/rules_ros2/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/mvukov/rules_ros2/actions/workflows/main.yml)

# Bazel rules for ROS2

This repo provides functionality to build and use ROS2 with Bazel.

## Prerequisites

You will need to install Bazel, see [here](https://docs.bazel.build/versions/master/install.html).
Besides Bazel, you will need a C++ compiler and a Python 3 interpreter.

And no, you don't have to install any ROS2 packages via `apt`.

The code is developed and tested on Ubuntu 20.04 with Python 3.8.

## What works?

Available features:

- Building of C++ and Python nodes (rclcpp and rclpy work).
- C/C++/Python code generation for interfaces (messages, services and actions).
- Defining ROS2 deployments with `ros2_launch` Bazel macro.
- Defining ROS2 tests with `ros2_test` Bazel macro.
- Only CycloneDDS middleware can be used at the moment.
  - Zero copy transport via shared memory backend ([iceoryx](https://github.com/eclipse-iceoryx/iceoryx)) for CycloneDDS.
- Utilities:
  - `ros2_param` for handling parameters
  - `ros2_topic` for handling topics

ROS2 packages are by default locked to versions from [release-humble-20221123](https://github.com/ros2/ros2/releases/tag/release-humble-20221123).

NOTE: Unlike ROS1 genmsg which refuses to generate code if the deps between
interface targets are not set correctly, code generation for ROS2 seems to not
care about this. If the deps are not correctly set, you'll only see failures
during compilation of the generated code. It could be that I missed setting some
flags to the code generation of ROS2 interfaces.

Please take a look at the [examples](examples) folder to get started.

## What's next?

The grand idea would be to get this repo at least to the level of
[rules_ros](https://github.com/mvukov/rules_ros). In particular:

- It has to be convenient to cross-compile a ROS2 deployment.

## Alternatives

For alternative approaches, see:

- [`ApexAI/rules_ros`](https://github.com/ApexAI/rules_ros/)
- [`RobotLocomotion/drake-ros/bazel_ros2_rules`](https://github.com/RobotLocomotion/drake-ros/tree/main/bazel_ros2_rules/ros2#alternatives),
  which includes a brief analysis of this and other approaches.
