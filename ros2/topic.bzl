""" Implements a macro for setting up a ROS 2 topic app.
"""

load("@rules_python//python:defs.bzl", "py_binary")

def ros2_topic(name, deps, **kwargs):
    """ Defines a ROS 2 topic app for a set of deps.

    Args:
        name: A unique target name.
        deps: A list of Python targets: typically IDL libraries
        (py_ros2_interface_library targets) used by the topic app.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    py_binary(
        name = name,
        srcs = ["@rules_ros2//ros2:ros2_topic.py"],
        main = "@rules_ros2//ros2:ros2_topic.py",
        deps = [
            "@rules_ros2//ros2:ros2_cmd",
            "@ros2cli//:ros2topic",
            "@ros2cli",
        ] + deps,
        **kwargs
    )
