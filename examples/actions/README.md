# ROS 2 Actions Example

This example implements a ROS 2 Action server and client. The server computes the Fibonacci sequence, and the client requests the sequence calculation.

## Purpose

- Demonstrate how to define, build, and use ROS 2 Actions.
- Show how to implement an Action Server and Action Client in C++ and Python.
- Illustrate the build rules required for generating Action interfaces.

## Dependencies

- `rules_ros2`: Core Bazel rules for ROS 2.
- `rclcpp`: ROS 2 Client Library for C++.
- `rclcpp_action`: Action implementation for `rclcpp`.
- `rclpy`: ROS 2 Client Library for Python.
- `action_msgs`: Standard action messages.

## Usage

You can run this example from the project root or as a standalone example from the `rules_ros2/examples` directory.

**Note:** Do not run these commands from the `rules_ros2/` directory itself.

### Option 1: Run from Project Root (`Perimeta_v2`)

Run the actions example (launches both server and client):

```bash
bazel run @rules_ros2//examples/actions:actions
```

To run individual components:

```bash
# C++ Server
bazel run @rules_ros2//examples/actions:server

# C++ Client
bazel run @rules_ros2//examples/actions:client

# Python Server
bazel run @rules_ros2//examples/actions:py_server
```

### Option 2: Run as Standalone (`rules_ros2/examples`)

Navigate to the examples directory:

```bash
cd rules_ros2/examples
```

Run the actions example:

```bash
bazel run //actions --experimental_isolated_extension_usages
```

To run individual components:

```bash
# C++ Server
bazel run //actions:server --experimental_isolated_extension_usages

# C++ Client
bazel run //actions:client --experimental_isolated_extension_usages

# Python Server
bazel run //actions:py_server --experimental_isolated_extension_usages
```
