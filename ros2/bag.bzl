""" Implements a macro for setting up a ROS 2 bag app.
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "py_launcher")
load("@com_github_mvukov_rules_ros2//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_binary")

def ros2_bag(name, idl_deps = None, **kwargs):
    """ Defines a binary target for a bag app.

    Args:
        name: A unique target name.
        idl_deps: Additional IDL deps that are used as runtime type-support plugins.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    launcher = "{}_launch".format(name)
    launch_script = py_launcher(
        launcher,
        deps = ["@ros2_rosbag2//:ros2bag"],
        idl_deps = idl_deps,
        template = "@com_github_mvukov_rules_ros2//ros2:bag.py.tpl",
        substitutions = {},
        tags = ["manual"],
    )

    deps = kwargs.pop("deps", [])
    py_binary(
        name = name,
        srcs = [launcher],
        main = launch_script,
        deps = [
            "@com_github_mvukov_rules_ros2//ros2:ros2_cmd",
            "@ros2_rosbag2//:ros2bag",
        ] + deps,
        **kwargs
    )
