""" Implements a macro for setting up a ROS 2 bag app.
"""

load("@com_github_mvukov_rules_ros2//ros2:py_defs.bzl", "ros2_py_binary")
load("@com_github_mvukov_rules_ros2//third_party:expand_template.bzl", "expand_template")

def ros2_bag(name, idl_deps = None, **kwargs):
    """ Defines a binary target for a bag app.

    Args:
        name: A unique target name.
        idl_deps: Additional IDL deps that are used as runtime type-support plugins.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    launch_py = "{}_launch_py".format(name)
    launch_script = launch_py + ".py"
    expand_template(
        name = launch_py,
        template = "@com_github_mvukov_rules_ros2//ros2:bag.py.tpl",
        out = launch_script,
        substitutions = {},
        tags = ["manual"],
    )

    deps = kwargs.pop("deps", [])
    ros2_py_binary(
        name = name,
        srcs = [launch_py],
        main = launch_script,
        set_up_ament = True,
        idl_deps = idl_deps,
        deps = [
            "@com_github_mvukov_rules_ros2//ros2:ros2_cmd",
            "@ros2_rosbag2//:ros2bag",
        ] + deps,
        **kwargs
    )
