# Zero-Copy Example

This example demonstrates ROS 2's capability for efficient inter-process communication using a shared memory backend, often referred to as "zero-copy" transport. It utilizes Iceoryx for shared memory and CycloneDDS as the RMW implementation.

## Purpose

- Demonstrate the benefits of zero-copy communication in ROS 2.
- Show how to configure Bazel to enable shared memory transport with CycloneDDS and Iceoryx.
- Illustrate the reduced latency and increased throughput for large messages using shared memory.

## Dependencies

- `rules_ros2`: Core Bazel rules for ROS 2.
- `rclcpp`: ROS 2 Client Library for C++.
- `rclrs`: ROS 2 Client Library for Rust.
- `iceoryx`: Eclipse Iceoryx, a high-performance inter-process communication (IPC) middleware.
- `cyclonedds`: Eclipse CycloneDDS, a high-performance DDS implementation.
- `std_msgs`: Standard ROS 2 message definitions.
- `libacl1-dev`: Required for Iceoryx to function correctly (install via `sudo apt-get install libacl1-dev` on Ubuntu).

## Usage

You can run this example from the project root or as a standalone example from the `rules_ros2/examples` directory.

**Note:** Do not run these commands from the `rules_ros2/` directory itself.

### Option 1: Run from Project Root (`Perimeta_v2`)

Run the zero-copy example with shared memory enabled:

```bash
bazel run @rules_ros2//examples/zero_copy:zero_copy -- @cyclonedds//:enable_shm=True
```

To run tests:

```bash
bazel test @rules_ros2//examples/zero_copy:tests -- @cyclonedds//:enable_shm=True
```

### Option 2: Run as Standalone (`rules_ros2/examples`)

Navigate to the examples directory:

```bash
cd rules_ros2/examples
```

Run the zero-copy example with shared memory enabled:

```bash
bazel run //zero_copy --experimental_isolated_extension_usages -- @cyclonedds//:enable_shm=True
```

To run tests:

```bash
bazel test //zero_copy:tests --experimental_isolated_extension_usages -- @cyclonedds//:enable_shm=True
```

### Observation

By tweaking the size of the data field in `Chatter.msg` file, you can observe that the delays for a e.g. a 4 MiB message and a 4 B message are similar when shared memory is enabled, highlighting the benefits of zero-copy transport.

### Tip

Add `build: --@cyclonedds//:enable_shm=True` to your `.bazelrc` file for convenience, so you don't have to specify it in every command.