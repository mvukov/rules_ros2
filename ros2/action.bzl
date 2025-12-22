""" Implements a macro for setting up a ROS 2 action app.
"""

load("@rules_python//python:defs.bzl", "py_binary")

def ros2_action(name, deps, **kwargs):
    """ Defines a ROS 2 action app for a set of deps.

    Args:
        name: A unique target name.
        deps: A list of Python targets: typically IDL libraries
        (py_ros2_interface_library targets) used by the action app.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    py_binary(
        name = name,
        srcs = ["@com_github_mvukov_rules_ros2//ros2:ros2_action.py"],
        main = "@com_github_mvukov_rules_ros2//ros2:ros2_action.py",
        deps = [
            "@com_github_mvukov_rules_ros2//ros2:ros2_cmd",
            "@ros2cli//:ros2action",
            "@ros2cli",
        ] + deps,
        **kwargs
    )
