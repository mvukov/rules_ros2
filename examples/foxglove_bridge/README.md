# Foxglove Bridge Example

This example demonstrates how to integrate your ROS 2 application with [Foxglove Studio](https://foxglove.dev/) using the Foxglove Bridge. It launches a sample publisher, a robot state publisher with a sample URDF, and the Foxglove Bridge node.

## Purpose

- Demonstrate visualization of ROS 2 data in Foxglove Studio.
- Show how to bundle and launch the Foxglove Bridge node using Bazel.
- Provide a setup for debugging and visualizing robot data (URDF, topics, logs).

<h2>Dependencies</h2>

- <code>rules_ros2</code>: Core Bazel rules for ROS 2.
- <code>foxglove_bridge</code>: The ROS 2 node for Foxglove connectivity.
- <code>ros2_robot_state_publisher</code>: Publishes the state of the robot to <code>tf</code>.
- <code>std_msgs</code>, <code>geometry_msgs</code>: Standard message definitions.

<h2>Usage</h2>

You can run this example from the project root or as a standalone example from the <code>rules_ros2/examples</code> directory.

<b>Note:</b> Do not run these commands from the <code>rules_ros2/</code> directory itself.

<h3>Option 1: Run from Project Root (<code>Perimeta_v2</code>)</h3>

Start the bridge and sample publisher:

<pre><code>bazel run @rules_ros2//examples/foxglove_bridge:foxglove_bridge
</code></pre>

<h3>Option 2: Run as Standalone (<code>rules_ros2/examples</code>)</h3>

Navigate to the examples directory:

<pre><code>cd rules_ros2/examples
</code></pre>

Start the bridge and sample publisher:

<pre><code>bazel run //foxglove_bridge --experimental_isolated_extension_usages
</code></pre>

<h3>Visualization with Foxglove Studio</h3>

<ol>
<li>Open <a href="https://studio.foxglove.dev/">Foxglove Studio</a> (or the desktop app).</li>
<li>Open a new connection.</li>
<li>Select <b>Foxglove WebSocket</b> as the connection type.</li>
<li>Set the URL to <code>ws://localhost:8765</code>.</li>
<li>Click <b>Open</b>.</li>
</ol>

<b>Recommended Panels:</b>
<ul>
<li><a href="https://foxglove.dev/docs/studio/panels/3d">3D</a>: To display the sample robot URDF model.</li>
<li><a href="https://foxglove.dev/docs/studio/panels/log">Log</a>: To display ROS log messages.</li>
<li><a href="https://foxglove.dev/docs/studio/panels/plot">Plot</a>: To plot data from topics (e.g., <code>/point</code>).</li>
</ul>