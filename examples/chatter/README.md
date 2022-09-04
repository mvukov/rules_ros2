# A chatter example

Implements simple talker and lister nodes.

To launch a sample deployment, run

```sh
bazel run //examples/chatter:launch
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
