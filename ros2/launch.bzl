""" Defines launch_ros-like ROS 2 deployment.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "ros2_ament_setup")
load("@com_github_mvukov_rules_ros2//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_binary")

def ros2_launch(name, nodes, launch_file, deps = None, data = None, **kwargs):
    """ Defines a ROS 2 deployment.

    Args:
        name: A unique target name.
        nodes: A list of ROS 2 nodes for the deployment.
        launch_file: A ros2launch-compatible launch file.
        deps: Additional Python deps that can be used by the launch file.
        data: Additional data that can be used by the launch file.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    if not nodes:
        fail("A list of nodes must be given!")

    data = data or []

    ament_setup_target = name + "_ament_setup"
    ament_setup_py_module = ros2_ament_setup(
        ament_setup_target,
        deps = nodes + data,
        tags = ["manual"],
    )

    substitutions = {
        "{launch_file}": "$(rootpath {})".format(launch_file),
        "{ament_setup}": ament_setup_py_module,
    }

    launch_script = "{}_launch.py".format(name)
    expand_template(
        name = "{}_launch_gen".format(name),
        template = "@com_github_mvukov_rules_ros2//ros2:launch.py.tpl",
        substitutions = substitutions,
        out = launch_script,
        data = [launch_file],
        tags = ["manual"],
    )

    deps = deps or []
    deps.append(ament_setup_target)
    py_binary(
        name = name,
        srcs = [launch_script],
        data = nodes + [launch_file] + data,
        main = launch_script,
        deps = [
            "@ros2_launch_ros//:ros2launch",
            "@ros2cli",
        ] + deps,
        **kwargs
    )
