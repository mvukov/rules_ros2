# A chatter example demonstrating shared memory backend for zero copy transport

ROS2 supports using shared memory to speed up transport of messages.

Before building, make sure you have `libacl1-dev` package installed on your system:

```sh
sudo apt-get install libacl1-dev
```

To run the example, start by running the shared memory manager:

```sh
bazel run --compilation_mode opt @iceoryx//:shared_memory_manager
```

Then, for each node started, specify the CycloneDDS config file:

```sh
export CYCLONEDDS_URI=file://<path to rules_ros2>/examples/zero_copy/cyclonedds.xml
bazel run --compilation_mode opt //examples/chatter:talker
```

In another terminal run

```sh
export CYCLONEDDS_URI=file://<path to rules_ros2>/examples/zero_copy/cyclonedds.xml
bazel run --compilation_mode opt //chatter:listener
```

You can see the delay for a 4 MB message is no different from a 4 bytes message.

You can run tests with

```sh
bazel test //chatter:tests  # To see the logs run with `--test_output=all`.
```
