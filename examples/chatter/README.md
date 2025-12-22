# Chatter Example

This example implements simple publisher ("talker") and subscriber ("listener") nodes using both `rclcpp` (C++) and `rclpy` (Python), demonstrating basic ROS 2 communication.

## Purpose

- Demonstrate basic publisher/subscriber communication.
- Show how to build and run C++ and Python ROS 2 nodes with Bazel.
- Provide a starting point for verifying your ROS 2 Bazel setup.

## Dependencies

- `rules_ros2`: Core Bazel rules for ROS 2.
- `rclcpp`: ROS 2 Client Library for C++.
- `rclpy`: ROS 2 Client Library for Python.
- `std_msgs`: Standard ROS 2 message definitions.

## Usage

You can run this example from the project root or as a standalone example from the `rules_ros2/examples` directory.

**Note:** Do not run these commands from the `rules_ros2/` directory itself.

### Option 1: Run from Project Root (`Perimeta_v2`)

Run the launch file to start both the talker and listener:

```bash
bazel run @rules_ros2//examples/chatter:chatter
```

To run individual nodes:

```bash
# C++ Talker
bazel run @rules_ros2//examples/chatter:talker

# Python Talker
bazel run @rules_ros2//examples/chatter:py_talker

# Listener (C++)
bazel run @rules_ros2//examples/chatter:listener
```

### Option 2: Run as Standalone (`rules_ros2/examples`)

Navigate to the examples directory:

```bash
cd rules_ros2/examples
```

Run the launch file:

```bash
bazel run //chatter --experimental_isolated_extension_usages
```

To run individual nodes:

```bash
# C++ Talker
bazel run //chatter:talker --experimental_isolated_extension_usages

# Python Talker
bazel run //chatter:py_talker --experimental_isolated_extension_usages

# Listener (C++)
bazel run //chatter:listener --experimental_isolated_extension_usages
```

### Additional Commands

Inspect the topic:
```bash
# Root
bazel run @rules_ros2//examples/chatter:topic -- echo /topic

# Standalone
bazel run //chatter:topic --experimental_isolated_extension_usages -- echo /topic
```

Record the topic:
```bash
# Root
bazel run @rules_ros2//examples/chatter:bag -- record /topic

# Standalone
bazel run //chatter:bag --experimental_isolated_extension_usages -- record /topic
```

Run tests:
```bash
# Root
bazel test @rules_ros2//examples/chatter:tests

# Standalone
bazel test //chatter:tests --experimental_isolated_extension_usages
```