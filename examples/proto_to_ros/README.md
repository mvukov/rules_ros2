# Proto2ros Example

This example demonstrates how to use `proto2ros` to convert Protocol Buffer
definitions to ROS 2 messages, integrated with working ROS 2 nodes.

## Overview

The example shows:

- Converting a `.proto` file to ROS 2 `.msg` format
- Using the `proto2ros_message` rule
- **Python talker node** publishing messages on the `chatter` topic
- **C++ listener node** subscribing to messages on the `chatter` topic

## Files

- `message.proto` - Protocol Buffer message definition
- `talker.py` - Python publisher node
- `listener.cc` - C++ subscriber node
- `proto2ros.py` - Launch file to run both nodes
- `BUILD.bazel` - Bazel build configuration

## Usage

### In BUILD.bazel

```python
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@com_github_mvukov_rules_ros2//ros2:proto2ros.bzl", "proto2ros_message")

proto_library(
    name = "chatter_proto",
    srcs = ["message.proto"],
)

proto2ros_message(
    name = "chatter_msgs",
    msg_names = ["ChatterMessage"],
    proto_library = ":chatter_proto",
)

ros2_interface_library(
    name = "chatter_interfaces",
    srcs = [":chatter_msgs"],
)
```

### Building

```bash
bazel build //examples/proto_to_ros:all
bazel build //examples/proto_to_ros:talker            # Python talker node
bazel build //examples/proto_to_ros:listener          # C++ listener node
bazel build //examples/proto_to_ros:proto_to_ros      # Launch file
```

### Running

Run the demo to see the Python talker and C++ listener communicate:

```bash
bazel run //examples/proto_to_ros:proto_to_ros
```
