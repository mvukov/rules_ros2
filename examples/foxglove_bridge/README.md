# Foxglove Bridge Example

This example demonstrates how to integrate your ROS 2 application with [Foxglove Studio](https://foxglove.dev/) using the Foxglove Bridge. It launches a sample publisher, a robot state publisher with a sample URDF, and the Foxglove Bridge node.

## Purpose

- Demonstrate visualization of ROS 2 data in Foxglove Studio.
- Show how to bundle and launch the Foxglove Bridge node using Bazel.
- Provide a setup for debugging and visualizing robot data (URDF, topics, logs).

## Dependencies

- `rules_ros2`: Core Bazel rules for ROS 2.
- `foxglove_bridge`: The ROS 2 node for Foxglove connectivity.
- `ros2_robot_state_publisher`: Publishes the state of the robot to `tf`.
- `std_msgs`, `geometry_msgs`: Standard message definitions.

## Usage

You can run this example from the project root or as a standalone example from the `rules_ros2/examples` directory.

**Note:** Do not run these commands from the `rules_ros2/` directory itself.

### Option 1: Run from Project Root (`Perimeta_v2`)

Start the bridge and sample publisher:

```bash
bazel run @rules_ros2//examples/foxglove_bridge:foxglove_bridge
```

### Option 2: Run as Standalone (`rules_ros2/examples`)

Navigate to the examples directory:

```bash
cd rules_ros2/examples
```

Start the bridge and sample publisher:

```bash
bazel run //foxglove_bridge --experimental_isolated_extension_usages
```

### Visualization with Foxglove Studio

1.  Open [Foxglove Studio](https://studio.foxglove.dev/) (or the desktop app).
2.  Open a new connection.
3.  Select **Foxglove WebSocket** as the connection type.
4.  Set the URL to `ws://localhost:8765`.
5.  Click **Open**.

**Recommended Panels:**
- [3D](https://foxglove.dev/docs/studio/panels/3d): To display the sample robot URDF model.
- [Log](https://foxglove.dev/docs/studio/panels/log): To display ROS log messages.
- [Plot](https://foxglove.dev/docs/studio/panels/plot): To plot data from topics (e.g., `/point`).
