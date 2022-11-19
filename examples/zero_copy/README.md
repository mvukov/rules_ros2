# A pub/sub example demonstrating shared memory backend for zero-copy transport

ROS2 supports using shared memory to speed up transport of messages.

Before building, make sure you have `libacl1-dev` package installed on your system.
You can install this library (licensed under GPL/LPGL) on Ubuntu as

```sh
sudo apt-get install libacl1-dev
```

To run the example run

```sh
bazel run //zero_copy --@cyclonedds//:enable_shm=True
```

By tweaking the size of the data field in Chatter.msg file, you can observe that
the delays for a e.g. a 4 MiB message and a 4 B message are similar.

You can run the corresponding simple test target with

```sh
bazel test //zero_copy:tests --@cyclonedds//:enable_shm=True  # To see the logs run with `--test_output=all`.
```

Tip: add line `build: --@cyclonedds//:enable_shm=True` to `.bazelrc`.
