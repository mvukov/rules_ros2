""" Defines launch_ros-like ROS 2 deployment.
"""

load("@com_github_mvukov_rules_ros2//ros2:py_defs.bzl", "ros2_py_binary")
load("@com_github_mvukov_rules_ros2//third_party:expand_template.bzl", "expand_template")

def ros2_launch(name, launch_file, nodes = None, deps = None, data = None, idl_deps = None, **kwargs):
    """ Defines a ROS 2 deployment.

    Args:
        name: A unique target name.
        launch_file: A ros2launch-compatible launch file.
        nodes: A list of ROS 2 nodes for the deployment.
        deps: Additional Python deps that can be used by the launch file.
        data: Additional data that can be used by the launch file.
        idl_deps: Additional IDL deps that are used as runtime plugins.
        **kwargs: Forwarded to the underlying ros2_py_binary; see
            https://bazel.build/reference/be/common-definitions#common-attributes-binaries.
    """
    launch_py = "{}_launch_py".format(name)
    nodes = nodes or []
    deps = deps or []
    launch_script = launch_py + ".py"
    expand_template(
        name = launch_py,
        template = "@com_github_mvukov_rules_ros2//ros2:ros2_launch.py.tpl",
        out = launch_script,
        substitutions = {
            "{launch_file}": "$(rootpath {})".format(launch_file),
        },
        data = [launch_file],
        tags = ["manual"],
    )

    data = data or []
    ros2_py_binary(
        name = name,
        srcs = [launch_py],
        main = launch_script,
        set_up_ament = True,
        idl_deps = idl_deps,
        data = nodes + [launch_file] + data,
        deps = [
            "@ros2_launch_ros//:ros2launch",
            "@ros2cli",
        ] + deps,
        **kwargs
    )
