# A chatter example

Implements simple talker and lister nodes.

To launch a sample deployment, run

```sh
bazel run //examples/chatter:launch
```
By running this command, Bazel will (re-)build all necessary code (ROS2, nodes,
launch relevant code, etc.) and run the target. This is basically one of the
value propositions for using Bazel: you don't have to (semi-)manually keep
e.g. nodes up-to-date. With correctly specified deps, Bazel takes care of this.

You can inspect the chatter topic with

```sh
bazel run //examples/chatter:topic -- echo /topic
```

You can run tests with

```sh
bazel test //examples/chatter:tests  # To get the logs run with `--test_output=all`.
```

Alternatively, you can run nodes without the launch mechanism. In a terminal run

```sh
bazel run //examples/chatter:talker
```
for rclcpp version or

```sh
bazel run //examples/chatter:py_talker
```
for rclpy version.


In another terminal run

```sh
bazel run //examples/chatter:listener
```
