""" Implements a macro for setting up a ROS 2 bag app.
"""

load("@com_github_mvukov_rules_ros2//ros2:py_defs.bzl", "ros2_py_binary")

_BAG_PY_SRC = "@com_github_mvukov_rules_ros2//ros2:ros2_bag.py"

def ros2_bag(name, idl_deps = None, **kwargs):
    """ Defines a binary target for a bag app.

    Args:
        name: A unique target name.
        idl_deps: Additional IDL deps that are used as runtime type-support plugins.
        **kwargs: Forwarded to the underlying ros2_py_binary; see
            https://bazel.build/reference/be/common-definitions#common-attributes-binaries.
    """
    deps = kwargs.pop("deps", [])
    ros2_py_binary(
        name = name,
        srcs = [_BAG_PY_SRC],
        main = _BAG_PY_SRC,
        set_up_ament = True,
        idl_deps = idl_deps,
        deps = [
            "@com_github_mvukov_rules_ros2//ros2:ros2_cmd",
            "@ros2_rosbag2//:ros2bag",
        ] + deps,
        **kwargs
    )
