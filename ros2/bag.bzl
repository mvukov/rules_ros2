""" Implements a macro for setting up a ROS2 bag app.
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "AMENT_SETUP_MODULE", "ros2_ament_setup")
load("@com_github_mvukov_rules_ros2//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_binary")

def ros2_bag(name, **kwargs):
    ament_setup_target = name + "_ament_setup"
    package_name = native.package_name()
    ros2_ament_setup(
        name = ament_setup_target,
        package_name = package_name,
        deps = ["@ros2_rosbag2//:ros2bag"],
    )

    repo_relative_ament_setup = paths.join(
        package_name,
        ament_setup_target,
        AMENT_SETUP_MODULE,
    ).replace("/", ".")
    substitutions = {
        "{ament_setup}": repo_relative_ament_setup,
    }

    main_script = "{}.py".format(name)
    expand_template(
        name = "{}_generator".format(name),
        template = "@com_github_mvukov_rules_ros2//ros2:bag.py.tpl",
        substitutions = substitutions,
        out = main_script,
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
