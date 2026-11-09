""" Implements a macro for setting up a ROS 2 bag app.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "split_kwargs")
load("@com_github_mvukov_rules_ros2//ros2:launcher.bzl", "ros2_launcher")
load("@rules_python//python:defs.bzl", "py_binary")

_BAG_MAIN = "@com_github_mvukov_rules_ros2//ros2:ros2_bag.py"

def ros2_bag(name, idl_deps = None, **kwargs):
    """ Defines a binary target for a bag app.

    Args:
        name: A unique target name.
        idl_deps: Additional IDL deps that are used as runtime type-support plugins.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    launcher_target_kwargs, binary_kwargs = split_kwargs(**kwargs)
    testonly = binary_kwargs.get("testonly", False)
    deps = binary_kwargs.pop("deps", [])

    target_impl = name + "_impl"
    py_binary(
        name = target_impl,
        srcs = [_BAG_MAIN],
        main = _BAG_MAIN,
        tags = ["manual"],
        deps = [
            "@com_github_mvukov_rules_ros2//ros2:ros2_cmd",
            "@ros2_rosbag2//:ros2bag",
        ] + deps,
        **binary_kwargs
    )

    ros2_launcher(
        name = name,
        entrypoint = target_impl,
        is_test = False,
        ament_setup_deps = [target_impl],
        idl_deps = idl_deps,
        testonly = testonly,
        **launcher_target_kwargs
    )
