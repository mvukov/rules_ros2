""" Defines ROS 2 testing functionality.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "ros2_ament_setup")
load("@com_github_mvukov_rules_ros2//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_test")
load("@rules_ros2_pip_deps//:requirements.bzl", "requirement")

def ros2_test(name, nodes, launch_file, deps = None, data = None, idl_deps = None, use_pytest = False, **kwargs):
    """ Defines a ROS 2 test.

    Args:
        name: A unique target name.
        nodes: A list of ROS 2 nodes in the test target.
        launch_file: A ros2test-compatible launch file.
        deps: Additional Python deps that can be used by the launch file.
        data: Additional data that can be used by the launch file.
        idl_deps: Additional IDL deps that are used as runtime plugins.
        use_pytest: If true, use pytest as the test driver.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    if not nodes:
        fail("A list of nodes must be given!")

    data = data or []

    ament_setup_target = name + "_ament_setup"
    ament_setup_py_module = ros2_ament_setup(
        ament_setup_target,
        deps = nodes + data,
        idl_deps = idl_deps,
        testonly = True,
        tags = ["manual"],
    )

    deps = deps or []
    deps.append(ament_setup_target)
    if use_pytest:
        _ros2_launch_pytest_test(name, nodes, launch_file, ament_setup_py_module, deps, data, **kwargs)
    else:
        _ros2_launch_testing_test(name, nodes, launch_file, ament_setup_py_module, deps, data, **kwargs)

def _ros2_launch_testing_test(name, nodes, launch_file, ament_setup_py_module, deps, data, **kwargs):
    substitutions = {
        "{launch_file}": "$(rootpath {})".format(launch_file),
        "{ament_setup}": ament_setup_py_module,
    }

    launch_script = "{}_launch.py".format(name)
    expand_template(
        name = "{}_launch_gen".format(name),
        template = "@com_github_mvukov_rules_ros2//ros2:test.py.tpl",
        substitutions = substitutions,
        out = launch_script,
        data = [launch_file],
        tags = ["manual"],
    )

    py_test(
        name = name,
        srcs = [launch_script],
        data = nodes + [launch_file] + data,
        main = launch_script,
        deps = [
            "@ros2_ament_cmake_ros//:domain_coordinator",
            "@ros2_launch//:launch_testing",
            "@ros2_launch_ros//:launch_testing_ros",
        ] + deps,
        **kwargs
    )

def _ros2_launch_pytest_test(name, nodes, launch_file, ament_setup_py_module, deps, data, **kwargs):
    launch_script = "{}_launch.py".format(name)
    expand_template(
        name = "{}_pytest_wrapper_gen".format(name),
        template = "@com_github_mvukov_rules_ros2//ros2:pytest_wrapper.py.tpl",
        substitutions = {"{ament_setup}": ament_setup_py_module},
        out = launch_script,
        tags = ["manual"],
    )

    py_test(
        name = name,
        srcs = [launch_script, launch_file],
        main = launch_script,
        data = nodes + data,
        args = kwargs.pop("args", []) + ["$(location :%s)" % launch_file],
        deps = deps + [
            "@ros2_launch//:launch_pytest",
            "@ros2_ament_cmake_ros//:domain_coordinator",
            requirement("coverage"),
            requirement("pytest"),
            requirement("pytest-cov"),
        ],
        **kwargs
    )
