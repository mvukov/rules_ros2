""" Defines commonly used Python macros.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "sh_launcher", "split_kwargs")
load("@com_github_mvukov_rules_ros2//third_party:symlink.bzl", "symlink")
load("@rules_python//python:defs.bzl", "py_binary", "py_test")

def _ros2_py_exec(target, is_test, name, srcs, main, set_up_ament, **kwargs):
    set_up_launcher = is_test or set_up_ament
    if set_up_launcher == False:
        target(name = name, srcs = srcs, main = main, **kwargs)
        return

    launcher_target_kwargs, binary_kwargs = split_kwargs(**kwargs)
    target_impl = name + "_impl"
    target(name = target_impl, srcs = srcs, main = main, tags = ["manual"], **binary_kwargs)

    target_impl_symlink = target_impl + "_symlink"
    symlink(
        name = target_impl_symlink,
        executable = target_impl,
        testonly = is_test,
        tags = ["manual"],
    )

    launcher = "{}_launch".format(name)
    ament_setup_deps = [target_impl] if set_up_ament else None
    sh_launcher(
        launcher,
        ament_setup_deps = ament_setup_deps,
        template = "@com_github_mvukov_rules_ros2//ros2:launch.sh.tpl",
        substitutions = {
            "{entry_point}": "$(rootpath {})".format(target_impl_symlink),
        },
        tags = ["manual"],
        data = [target_impl_symlink],
        testonly = is_test,
    )

    sh_target = native.sh_test if is_test else native.sh_binary
    sh_target(
        name = name,
        srcs = [launcher],
        data = [target_impl_symlink],
        **launcher_target_kwargs
    )

def ros2_py_binary(name, srcs, main, set_up_ament = False, target_rule = None, **kwargs):
    """ Defines a ROS 2 Python binary.

    Args:
        name: A unique target name.
        srcs: List of source files.
        main: Source file to use as entrypoint.
        set_up_ament: If true, sets up ament file tree for the binary target.
        target_rule: `py_binary` rule to use (defaults to rules_python implementation).
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    if not target_rule:
        target_rule = py_binary
    is_test = False
    _ros2_py_exec(target_rule, is_test, name, srcs, main, set_up_ament, **kwargs)

def ros2_py_test(name, srcs, main, set_up_ament = True, target_rule = None, **kwargs):
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
        target_rule: `py_test` rule to use (defaults to rules_python implementation).
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    if not target_rule:
        target_rule = py_test
    is_test = True
    _ros2_py_exec(target_rule, is_test, name, srcs, main, set_up_ament, **kwargs)
