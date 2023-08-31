""" Defines launch_ros-like ROS 2 deployment.
"""

load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_ros2//ros2:ament.bzl", "py_launcher")

def ros2_launch(name, launch_file, nodes = None, deps = None, data = None, idl_deps = None, **kwargs):
    """ Defines a ROS 2 deployment.

    Args:
        name: A unique target name.
        launch_file: A ros2launch-compatible launch file.
        nodes: A list of ROS 2 nodes for the deployment.
        deps: Additional Python deps that can be used by the launch file.
        data: Additional data that can be used by the launch file.
        idl_deps: Additional IDL deps that are used as runtime plugins.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    launcher = "{}_launch".format(name)
    nodes = nodes or []
    deps = deps or []
    launch_script = py_launcher(
        launcher,
        deps = nodes + deps,
        idl_deps = idl_deps,
        template = "@rules_ros2//ros2:launch.py.tpl",
        substitutions = {
            "{launch_file}": "$(rootpath {})".format(launch_file),
        },
        data = [launch_file],
        tags = ["manual"],
    )

    data = data or []
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
