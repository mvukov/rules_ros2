""" Defines ROS 2 testing functionality.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "py_launcher")
load("@rules_python//python:defs.bzl", "py_test")
load("@rules_ros2_pip_deps//:requirements.bzl", "requirement")

def ros2_test(name, launch_file, nodes = None, deps = None, data = None, idl_deps = None, use_pytest = False, **kwargs):
    """ Defines a ROS 2 test.

    In case you don't need ROS 2 nodes for tests, but need ament setup such
    that e.g. plugins can work: use a lightweight macro ros2_cpp_test
    from //ros2:cc_defs.bzl.

    Args:
        name: A unique target name.
        launch_file: A ros2test-compatible launch file.
        nodes: A list of ROS 2 nodes in the test target.
        deps: Additional Python deps that can be used by the launch file.
        data: Additional data that can be used by the launch file.
        idl_deps: Additional IDL deps that are used as runtime plugins.
        use_pytest: If true, use pytest as the test driver.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    nodes = nodes or []
    data = data or []
    deps = deps or []
    if use_pytest:
        _ros2_launch_pytest_test(name, nodes, launch_file, deps, data, idl_deps, **kwargs)
    else:
        _ros2_launch_testing_test(name, nodes, launch_file, deps, data, idl_deps, **kwargs)

def _ros2_launch_testing_test(name, nodes, launch_file, deps, data, idl_deps, **kwargs):
    launcher = "{}_launch".format(name)
    launch_script = py_launcher(
        launcher,
        deps = nodes + deps,
        idl_deps = idl_deps,
        template = "@com_github_mvukov_rules_ros2//ros2:test.py.tpl",
        substitutions = {
            "{launch_file}": "$(rootpath {})".format(launch_file),
        },
        data = [launch_file],
        testonly = True,
        tags = ["manual"],
    )

    py_test(
        name = name,
        srcs = [launcher],
        data = nodes + [launch_file] + data,
        main = launch_script,
        deps = [
            "@ros2_launch//:launch_testing",
            "@ros2_launch_ros//:launch_testing_ros",
        ] + deps,
        **kwargs
    )

def _ros2_launch_pytest_test(name, nodes, launch_file, deps, data, idl_deps, **kwargs):
    launcher = "{}_launch".format(name)
    launch_script = py_launcher(
        launcher,
        deps = nodes + deps,
        idl_deps = idl_deps,
        template = "@com_github_mvukov_rules_ros2//ros2:pytest_wrapper.py.tpl",
        substitutions = {},
        testonly = True,
        tags = ["manual"],
    )

    py_test(
        name = name,
        srcs = [launcher, launch_file],
        main = launch_script,
        data = nodes + data,
        args = kwargs.pop("args", []) + ["$(rootpath :%s)" % launch_file],
        deps = [
            "@ros2_launch//:launch_pytest",
            requirement("coverage"),
            requirement("pytest"),
            requirement("pytest-cov"),
        ] + deps,
        **kwargs
    )
