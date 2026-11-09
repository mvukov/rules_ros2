""" Defines ROS 2 testing functionality.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "split_kwargs")
load("@com_github_mvukov_rules_ros2//ros2:launcher.bzl", "ros2_launcher")
load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_ros2_pip_deps//:requirements.bzl", "requirement")

_LAUNCH_TESTING_MAIN = "@com_github_mvukov_rules_ros2//ros2:ros2_launch_testing.py"
_PYTEST_MAIN = "@com_github_mvukov_rules_ros2//ros2:pytest_wrapper.py"

def ros2_test(name, launch_file, nodes = None, deps = None, data = None, idl_deps = None, use_pytest = False, **kwargs):
    """ Defines a ROS 2 test.

    For lighter options, e.g. you don't need a launch file, please take a look at:
    * ros2_cpp_test in //ros2:cc_defs.bzl and
    * ros2_py_test in //ros2:py_defs.bzl

    Defaults ROS_HOME and ROS_LOG_DIR to $TEST_UNDECLARED_OUTPUTS_DIR (if set,
    otherwise to $TEST_TMPDIR, see https://bazel.build/reference/test-encyclopedia#initial-conditions)

    Please make sure that --sandbox_default_allow_network=false is set in .bazelrc.
    This ensures proper network isolation.

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
        main = _PYTEST_MAIN
        srcs = [main, launch_file]
        impl_data = nodes + data
        impl_deps = [
            "@ros2_launch//:launch_pytest",
            requirement("coverage"),
            requirement("pytest"),
            requirement("pytest-cov"),
        ]
    else:
        main = _LAUNCH_TESTING_MAIN
        srcs = [main]
        impl_data = nodes + [launch_file] + data
        impl_deps = [
            "@ros2_launch//:launch_testing",
            "@ros2_launch_ros//:launch_testing_ros",
        ]

    launcher_target_kwargs, test_kwargs = split_kwargs(**kwargs)
    target_impl = name + "_impl"
    py_binary(
        name = target_impl,
        srcs = srcs,
        main = main,
        data = impl_data,
        tags = ["manual"],
        testonly = True,
        deps = impl_deps + deps,
        **test_kwargs
    )

    # The launch file is passed as the first argument to the test driver.
    ros2_launcher(
        name = name,
        entrypoint = target_impl,
        is_test = True,
        ament_setup_deps = nodes + deps,
        idl_deps = idl_deps,
        runfile_args = [launch_file],
        **launcher_target_kwargs
    )
