""" Implements a macro for setting up a ROS 2 bag app.
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "ros2_ament_setup")
load("@com_github_mvukov_rules_ros2//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_binary")

def ros2_bag(name, idl_deps = None, **kwargs):
    """ Defines a binary target for a bag app.

    Args:
        name: A unique target name.
        idl_deps: Additional IDL deps that are used as runtime type-support plugins.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    ament_setup_target = name + "_ament_setup"
    ament_setup_py_module = ros2_ament_setup(
        ament_setup_target,
        deps = ["@ros2_rosbag2//:ros2bag"],
        idl_deps = idl_deps,
        tags = ["manual"],
    )

    main_script = "{}.py".format(name)
    expand_template(
        name = "{}_generator".format(name),
        template = "@com_github_mvukov_rules_ros2//ros2:bag.py.tpl",
        substitutions = {
            "{ament_setup}": ament_setup_py_module,
        },
        out = main_script,
        tags = ["manual"],
    )

    deps = kwargs.pop("deps", []) + [ament_setup_target]
    py_binary(
        name = name,
        srcs = [main_script],
        main = main_script,
        deps = [
            "@com_github_mvukov_rules_ros2//ros2:ros2_cmd",
            "@ros2_rosbag2//:ros2bag",
        ] + deps,
        **kwargs
    )
