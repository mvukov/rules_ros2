[![CI](https://github.com/mvukov/rules_ros2/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/mvukov/rules_ros2/actions/workflows/main.yml)

# Bazel rules for ROS2

This repo aims to build ROS2 from scratch with Bazel.

## Prerequisites

You will need to install Bazel, see [here](https://docs.bazel.build/versions/master/install.html).
Besides Bazel, you will need a C++ compiler and a Python 3 interpreter.

And no, you don't have to install any ROS2 packages via `apt`.

The code is developed and tested on Ubuntu 20.04 with Python 3.8.

## What works?

At the moment it is supported:

- Building of C++ and Python nodes (rclcpp and rclpy work).
- C/C++/Python code generation for interfaces (messages, services and actions).
- Defining ROS2 deployments with `ros2_launch` Bazel macro.
- Defining ROS2 tests with `ros2_test` Bazel macro.
- Only CycloneDDS middleware can be interfaced at the moment.
- Zero copy transport via shared memory backend ([iceoryx](https://github.com/eclipse-iceoryx/iceoryx)) for CycloneDDS.

ROS2 packages are by default locked to versions from [release-humble-20220523](https://github.com/ros2/ros2/releases/tag/release-humble-20220523).

NOTE: Unlike ROS1 genmsg which refuses to generate code if the deps between
interface targets are not set correctly, code generation for ROS2 seems to not
care about this. If the deps are not correctly set, you'll only see failures
during compilation of the generated code. It could be that I missed setting some
flags to the code generation of ROS2 interfaces.

Please take a look at the examples folder to get more info.

## What's next?

The grand idea would be to get this repo at least to the level of
[rules_ros](https://github.com/mvukov/rules_ros). In particular:

- It has to be convenient to cross-compile a ROS2 deployment.
