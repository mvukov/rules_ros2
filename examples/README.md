# Examples for rules_ros2

This folder provides some examples and guidelines on how you can use this repo
in your very own (mono)repo.

## Handling of external dependencies

It's important to note here that this folder is
a yet another [repository](https://bazel.build/concepts/build-ref#repositories).
Regarding handling of [external dependencies](https://bazel.build/external/overview),
there are two ways to set up your repo to use this repo. In the following it is
explained how to set up each of them with some pros and cons.

### WORKSPACE-file based setup

This is [traditional](https://bazel.build/versions/7.5.0/external/overview#workspace-system) way of setting up external dependencies that Bazel-team plans to phase out.
Please take a look at the `WORKSPACE` file in this folder to get an idea how to set up yourself.

This approach allows one to customize a Python interpreter and Python deps
in `requirements.txt` file. In particular, the user is responsible to define the interpreter
and requirements lock file in their repo. The official interpreter version for ROS 2 Humble is 3.10
and in `WORKSPACE` file a 3.10 interpreter is setup up. Please pay attention that
resolved dependencies in `requirements_lock.txt` file are resolved for Python 3.10.

If you want to use this approach, make sure you have the following lines in your `.bazelrc` file:

```bash
common --noenable_bzlmod --enable_workspace
```

### [bzlmod](https://bazel.build/versions/7.5.0/external/overview#bzlmod)-based setup

This is the new external dependency subsystem. Please take a look at the `MODULE.bazel` file in this folder to get an idea how to set up yourself.

With this approach, `rules_ros2` defines a Python 3.10 interpreter and defines the hub with resolved Python deps. If you want a different intepreter and/or you want to extend the Python requirements files with extra deps you might want to use in your monorepo, you'll need to patch `requirements_lock.txt` file and this repo.

This is the default way for handling external deps since Bazel 7.

If you want to be more explicit and explicitly disable the workspace-based setup, you can add the following lines in your `.bazelrc` file:

```bash
common --enable_bzlmod --noenable_workspace
```

## The rest of the setup

Besides setting up your workspace/bzlmod file, please make sure you copy the `.bazelrc`
file as well to your monorepo -- or at least adjust your own using the provided
one.

Before you build/run/test any of the targets in this folder, please make sure
your terminal is in this folder, or in a subfolder, and not in the root
of the repo.

You can start with a simple [chatter](chatter) example.
