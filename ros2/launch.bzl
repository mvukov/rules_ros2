""" Defines launch_ros-like ROS 2 deployment.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "split_kwargs")
load("@com_github_mvukov_rules_ros2//ros2:launcher.bzl", "ros2_launcher")
load("@rules_python//python:defs.bzl", "py_binary")

_LAUNCH_MAIN = "@com_github_mvukov_rules_ros2//ros2:ros2_launch.py"

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
    nodes = nodes or []
    deps = deps or []
    data = data or []
    launcher_target_kwargs, binary_kwargs = split_kwargs(**kwargs)
    testonly = binary_kwargs.get("testonly", False)

    target_impl = name + "_impl"
    py_binary(
        name = target_impl,
        srcs = [_LAUNCH_MAIN],
        main = _LAUNCH_MAIN,
        data = nodes + [launch_file] + data,
        tags = ["manual"],
        deps = [
            "@ros2_launch_ros//:ros2launch",
            "@ros2cli",
        ] + deps,
        **binary_kwargs
    )

    ros2_launcher(
        name = name,
        entrypoint = target_impl,
        is_test = False,
        ament_setup_deps = nodes + deps,
        idl_deps = idl_deps,
        runfile_args = [launch_file],
        testonly = testonly,
        **launcher_target_kwargs
    )
