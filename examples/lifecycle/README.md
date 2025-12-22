# Lifecycle Node Example

This example demonstrates the use of managed lifecycle nodes in ROS 2. It includes a lifecycle talker, a lifecycle listener, and a service client to manage the state transitions.

## Purpose

- Demonstrate how to implement and manage Lifecycle Nodes.
- Show how to trigger state transitions (Configure, Activate, Deactivate, Cleanup, Shutdown) using a service client.
- Provide a reference for building more complex, state-managed ROS 2 applications.

## Dependencies

- `rules_ros2`: Core Bazel rules for ROS 2.
- `rclcpp`: ROS 2 Client Library for C++.
- `rclcpp_lifecycle`: Lifecycle node interface for `rclcpp`.
- `std_msgs`: Standard ROS 2 message definitions.
- `lifecycle_msgs`: Messages and services for lifecycle management.

## Usage

You can run this example from the project root or as a standalone example from the `rules_ros2/examples` directory.

**Note:** Do not run these commands from the `rules_ros2/` directory itself.

### Option 1: Run from Project Root (`Perimeta_v2`)

Run the lifecycle deployment:

```bash
bazel run @rules_ros2//examples/lifecycle:lifecycle
```

You can also interact with the lifecycle nodes using the lifecycle utility:

```bash
bazel run @rules_ros2//:ros2_lifecycle
```

### Option 2: Run as Standalone (`rules_ros2/examples`)

Navigate to the examples directory:

```bash
cd rules_ros2/examples
```

Run the lifecycle deployment:

```bash
bazel run //lifecycle --experimental_isolated_extension_usages
```

Interact using the lifecycle utility (target name may vary depending on local alias, typically `ros2_lifecycle` is top-level):

```bash
bazel run //:ros2_lifecycle --experimental_isolated_extension_usages
```

### Reference

For detailed information on ROS 2 Lifecycle nodes, look [here](https://github.com/ros2/demos/tree/0.20.2/lifecycle).