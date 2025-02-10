""" Implements a macro for setting up a ROS 2 service app.
"""

load("@rules_python//python:defs.bzl", "py_binary")

def ros2_service(name, deps, **kwargs):
    """ Defines a ROS 2 service app for a set of deps.

    Args:
        name: A unique target name.
        deps: A list of Python targets: typically IDL libraries
        (py_ros2_interface_library targets) used by the service app.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    py_binary(
        name = name,
        srcs = ["@com_github_mvukov_rules_ros2//ros2:ros2_service.py"],
        main = "@com_github_mvukov_rules_ros2//ros2:ros2_service.py",
        deps = [
            "@com_github_mvukov_rules_ros2//ros2:ros2_cmd",
            "@ros2cli//:ros2service",
            "@ros2cli",
        ] + deps,
        **kwargs
    )
