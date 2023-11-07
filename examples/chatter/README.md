# A chatter example

Implements simple talker and lister nodes.

To launch a sample deployment, run

```sh
bazel run //chatter
```

By running this command, Bazel will (re-)build all necessary code (ROS 2, nodes,
launch relevant code, etc.) and run the target. This is basically one of the
value propositions for using Bazel: you don't have to (semi-)manually keep
e.g. nodes up-to-date. With correctly specified deps, Bazel takes care of this.

You can inspect the chatter topic with

```sh
bazel run //chatter:topic -- echo /topic
```

and list the parameters with

```sh
bazel run //:ros2_param -- list
```

You can record a topic with

```sh
bazel run //chatter:bag -- record /topic
```

You can run tests with

```sh
bazel test //chatter:tests  # To see the logs run with `--test_output=all`.
```

To run the sample deployment in a Docker container, run

```sh
bazel run //chatter:chatter_image
```

Alternatively, you can run nodes without the launch mechanism. In a terminal run

```sh
bazel run //chatter:talker
```

for rclcpp version or

```sh
bazel run //chatter:py_talker
```

for rclpy version.

In another terminal run

```sh
bazel run //chatter:listener
```
