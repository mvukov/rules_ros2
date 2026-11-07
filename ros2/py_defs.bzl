""" Defines commonly used Python macros.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "split_kwargs")
load("@com_github_mvukov_rules_ros2//ros2:launcher.bzl", "ros2_launcher")
load("@rules_python//python:defs.bzl", "py_binary")

def _ros2_py_exec(target, name, srcs, main, set_up_ament, testonly, **kwargs):
    set_up_launcher = testonly or set_up_ament
    if set_up_launcher == False:
        target(name = name, srcs = srcs, main = main, **kwargs)
        return

    launcher_target_kwargs, binary_kwargs = split_kwargs(**kwargs)
    target_impl = name + "_impl"
    target(
        name = target_impl,
        srcs = srcs,
        main = main,
        tags = ["manual"],
        testonly = testonly,
        **binary_kwargs
    )

    ros2_launcher(
        name = name,
        entrypoint = target_impl,
        is_test = testonly,
        ament_setup_deps = [target_impl] if set_up_ament else None,
        testonly = testonly,
        **launcher_target_kwargs
    )

def ros2_py_binary(name, srcs, main, set_up_ament = False, **kwargs):
    """ Defines a ROS 2 Python binary.

    Args:
        name: A unique target name.
        srcs: List of source files.
        main: Source file to use as entrypoint.
        set_up_ament: If true, sets up ament file tree for the binary target.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    _ros2_py_exec(
        py_binary,
        name,
        srcs,
        main,
        set_up_ament,
        testonly = False,
        **kwargs
    )

def ros2_py_test(name, srcs, main, set_up_ament = True, **kwargs):
    """ Defines a ROS 2 Python test.

    Defaults ROS_HOME and ROS_LOG_DIR to $TEST_UNDECLARED_OUTPUTS_DIR (if set,
    otherwise to $TEST_TMPDIR, see https://bazel.build/reference/test-encyclopedia#initial-conditions)

    Please make sure that --sandbox_default_allow_network=false is set in .bazelrc.
    This ensures proper network isolation.

    Args:
        name: A unique target name.
        srcs: List of source files.
        main: Source file to use as entrypoint.
        set_up_ament: If true, sets up ament file tree for the test target.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    _ros2_py_exec(
        py_binary,
        name,
        srcs,
        main,
        set_up_ament,
        testonly = True,
        **kwargs
    )

def ros2_py_exec(name, srcs, main, rule_impl, set_up_ament, testonly, **kwargs):
    """ Defines a ROS 2 Python target using a user-specified rule.

    Args:
        name: A unique target name.
        srcs: List of source files.
        main: Source file to use as entrypoint.
        rule_impl: The target rule to use (e.g., `py_binary`).
        testonly: Set to True if this is a test target; False otherwise.
        set_up_ament: If true, generate a launcher for the target which:
            * Sets AMENT_PREFIX_PATH to point to a generated ament file tree
            * Defaults ROS_HOME and ROS_LOG_DIR to $TEST_UNDECLARED_OUTPUTS_DIR (if set,
              otherwise to $TEST_TMPDIR, see https://bazel.build/reference/test-encyclopedia#initial-conditions)
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    _ros2_py_exec(
        target = rule_impl,
        name = name,
        srcs = srcs,
        main = main,
        set_up_ament = set_up_ament,
        testonly = testonly,
        **kwargs
    )
