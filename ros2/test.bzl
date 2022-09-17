""" Defines launch_ros-like ROS2 deployment.
"""

load("@com_github_mvukov_rules_ros2//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_test")

def ros2_test(name, nodes, launch_file, deps = None, data = None, **kwargs):
    """ Defines a ROS2 deployment.

    Args:
        name: A unique target name.
        nodes: A list of ROS2 nodes for the deployment.
        launch_file: A roslaunch-compatible launch file.
        deps: Additional Python deps that can be used by the launch file.
        data: Additional data that can be used bu the launch file.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    if not nodes:
        fail("A list of nodes must be given!")

    substitutions = {
        "{launch_file}": "$(rootpath {})".format(launch_file),
    }

    launch_script = "{}_launch.py".format(name)
    expand_template(
        name = "{}_launch_gen".format(name),
        template = "@com_github_mvukov_rules_ros2//ros2:test.py.tpl",
        substitutions = substitutions,
        out = launch_script,
        data = [launch_file],
    )

    deps = deps or []
    data = data or []
    py_test(
        name = name,
        srcs = [launch_script],
        data = nodes + [launch_file] + data,
        main = launch_script,
        deps = [
            "@ros2_ros_testing//:ros2test",
            "@ros2cli",
        ] + deps,
        **kwargs
    )
