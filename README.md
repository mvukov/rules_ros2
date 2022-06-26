# Bazel rules for ROS2

This repo aims to build ROS2 from scratch with Bazel.

## Prerequisites

The code is developed and tested on Ubuntu 20.04 with Python 3.8.

You will need to install Bazel, see [here](https://docs.bazel.build/versions/master/install.html).
Besides Bazel, you will need a C++ compiler and a Python 3.8 interpreter.

And no, you don't have to install any ROS2 packages via `apt`.

## What works?

At the moment it is only supported to build C++ nodes.
ROS2 package versions target ROS2 Foxy.
In particular:

- C/C++ code generation for interfaces (messages, services and actions) works.
- rclcpp can be compiled.
- Only CycloneDDS middleware can be interfaced at the moment.

NOTE: Unlike ROS1 genmsg which refuses to generate code if the deps between
interface targets are not set correctly, code generation for ROS2 seems to not
care about this. If the deps are not correctly set, you'll only see failures
during compilation of the generated code. It could be that I missed setting some
flags to the (quite involved) code generation of ROS2 interfaces.

Please take a look at the examples folder to get more info.

## What's next?

The next step would be to work out code-generation of interfaces for Python.
Compared to ROS1 this is quite more involved as in ROS2 Python interfaces have
both C/C++ and Python code.
That is the prerequisite to get ROS2 launch system working. Compared to ROS1
where roslaunch is independent of rospy, in ROS2 launch_ros and rclpy are tighly coupled. So, no matter whether you only build C/C++ nodes, you also need to have substantial amount of Python stuff in your deployment.

The grand idea would be to get this repo at least to the level of
[rules_ros](https://github.com/mvukov/rules_ros). In particular:
- It has to be simple & convenient to assemble a deployment, i.e. a Bazel target with a launch mechanism and all relevant nodes.
- It has to be simple & convenient to cross-compile a ROS2 deployment.
