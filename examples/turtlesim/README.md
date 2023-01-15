# The turtlesim example from ROS tutorial

This is the well known [turtlesim example of ROS tutorial](http://wiki.ros.org/turtlesim)

Since `ament_index_cpp::get_package_share_director` is not yet working properly, the path to the turtle images needs to be configured manually, to that end, addadot examples/turtlesim/src/turtle_frame.cpp#L3.

To launch the turtlesim main window, run

```sh
bazel run //turtlesim
```

In order to start the teleoperator, execute
```sh
bazel run //turtlesim:teleop
```

You can now remote control the turtle in the turtlesim window from the teleop shell.
