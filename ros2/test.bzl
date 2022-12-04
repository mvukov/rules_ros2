""" Defines ROS2 testing functionality.
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "AMENT_SETUP_MODULE", "ros2_ament_setup")
load("@com_github_mvukov_rules_ros2//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_test")

def ros2_test(name, nodes, launch_file, deps = None, data = None, **kwargs):
    """ Defines a ROS2 test.

    Args:
        name: A unique target name.
        nodes: A list of ROS2 nodes in the test target.
        launch_file: A ros2test-compatible launch file.
        deps: Additional Python deps that can be used by the launch file.
        data: Additional data that can be used by the launch file.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    if not nodes:
        fail("A list of nodes must be given!")

    ament_setup_target = name + "_ament_setup"
    package_name = native.package_name()
    ros2_ament_setup(
        name = ament_setup_target,
        package_name = package_name,
        deps = nodes,
        testonly = True,
    )

    repo_relative_ament_setup = paths.join(
        package_name,
        ament_setup_target,
        AMENT_SETUP_MODULE,
    ).replace("/", ".")
    substitutions = {
        "{launch_file}": "$(rootpath {})".format(launch_file),
        "{ament_setup}": repo_relative_ament_setup,
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
    deps.append(ament_setup_target)
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
