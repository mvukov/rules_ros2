""" Defines launch_ros-like ROS 2 deployment.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "py_launcher")
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
    launcher = "{}_launch".format(name)
    launch_script = py_launcher(
        launcher,
        deps = nodes + data,
        template = "@com_github_mvukov_rules_ros2//ros2:launch.py.tpl",
        substitutions = {
            "{launch_file}": "$(rootpath {})".format(launch_file),
        },
        data = [launch_file],
        tags = ["manual"],
    )

    deps = deps or []
    py_binary(
        name = name,
        srcs = [launcher],
        data = nodes + [launch_file] + data,
        main = launch_script,
        deps = [
            "@ros2_launch_ros//:ros2launch",
            "@ros2cli",
        ] + deps,
        **kwargs
    )
