# A chatter example demonstrating shared memory backend for zero copy transport

ROS2 supports using shared memory to speed up transport of messages. First, run shared memory manager:

```sh
bazel run @iceoryx//:shared_memory_manager
```

Then, for each node started, specify the CycloneDDS config file:

```sh
export CYCLONEDDS_URI=file://<path to rules_ros2>/examples/configs/cyclonedds.xml
bazel run //examples/chatter:talker
```

In another terminal run

```sh
bazel ruexport CYCLONEDDS_URI=file://<path to rules_ros2>/examples/configs/cyclonedds.xml
bazel run //chatter:listener
```

You can run tests with

```sh
bazel test //chatter:tests  # To see the logs run with `--test_output=all`.
```
