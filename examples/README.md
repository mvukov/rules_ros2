# Examples for rules_ros2

This folder provides some examples and guidelines on how you can use this repo
in your very own (mono)repo. It's important to note here that this folder is
a yet another Bazel workspace (it has a WORKSPACE file). Please take a look at
the workspace file to get an idea how to set up yourself.

Besides setting up your workspace file, please make sure you copy the `.bazelrc`
file as well to your monorepo -- or at least adjust your own using the provided
one.

Before you build/run/test any of the targets in this folder, please make sure
your terminal is in this folder, or in a subfolder, and not in the root
of the repo.

If you want to run ROS2 deployments in Docker containers, please install
[Docker](https://docs.docker.com/engine/install/ubuntu/).

You can start with a simple [chatter](chatter) example.
