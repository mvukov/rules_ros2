# Bazel rules for ROS2

This repo aims to build ROS2 from scratch with Bazel.

## Prerequisites

The code is developed and tested on Ubuntu 20.04 with Python 3.8.

You will need to install Bazel, see [here](https://docs.bazel.build/versions/master/install.html).
Besides Bazel, you will need a C++ compiler and a Python 3.8 interpreter.
If you want to run ROS2 deployments in Docker containers, [install Docker](https://docs.docker.com/engine/install/ubuntu/)
as well.

And no, you don't have to install any ROS2 packages via `apt`.

## What works?

At the moment it is supported:

- Building of C++ and Python nodes (rclcpp and rclpy work).
- C/C++/Python code generation for interfaces (messages, services and actions).
- Defining ROS2 deployments with `ros2_launch` Bazel macro.
- Defining ROS2 tests with `ros2_test` Bazel macro.
- Only CycloneDDS middleware can be interfaced at the moment.

ROS2 packages are by default locked to versions from [release-foxy-20220928](https://github.com/ros2/ros2/releases/tag/release-foxy-20220928).

NOTE: Unlike ROS1 genmsg which refuses to generate code if the deps between
interface targets are not set correctly, code generation for ROS2 seems to not
care about this. If the deps are not correctly set, you'll only see failures
during compilation of the generated code. It could be that I missed setting some
flags to the code generation of ROS2 interfaces.

Please take a look at the examples folder to get more info. Start with a simple
[chatter](examples/chatter) example.

## What's next?

The grand idea would be to get this repo at least to the level of
[rules_ros](https://github.com/mvukov/rules_ros). In particular:

- It has to be convenient to cross-compile a ROS2 deployment.
