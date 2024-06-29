""" Defines commonly used Python macros.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "sh_launcher", "split_kwargs")
load("@com_github_mvukov_rules_ros2//third_party:symlink.bzl", "symlink")
load("@rules_ros2_pythons//3.10:defs.bzl", "py_binary", "py_test")

def _ros2_py_exec(target, name, srcs, main, set_up_ament, **kwargs):
    if set_up_ament == False:
        target(name = name, srcs = srcs, main = main, **kwargs)
        return

    launcher_target_kwargs, binary_kwargs = split_kwargs(**kwargs)
    target_impl = name + "_impl"
    target(name = target_impl, srcs = srcs, main = main, tags = ["manual"], **binary_kwargs)

    is_test = target == py_test

    target_impl_symlink = target_impl + "_symlink"
    symlink(
        name = target_impl_symlink,
        executable = target_impl,
        testonly = is_test,
        tags = ["manual"],
    )

    launcher = "{}_launch".format(name)
    sh_launcher(
        launcher,
        deps = [target_impl],
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

def ros2_py_binary(name, srcs, main, set_up_ament = False, **kwargs):
    """ Defines a ROS 2 Python binary.

    Args:
        name: A unique target name.
        srcs: List of source files.
        main: Source file to use as entrypoint.
        set_up_ament: If true, sets up ament file tree for the binary target.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    _ros2_py_exec(py_binary, name, srcs, main, set_up_ament, **kwargs)

def ros2_py_test(name, srcs, main, set_up_ament = True, **kwargs):
    """ Defines a ROS 2 Python test.

    Args:
        name: A unique target name.
        srcs: List of source files.
        main: Source file to use as entrypoint.
        set_up_ament: If true, generate a launcher for the target which:
            * Sets AMENT_PREFIX_PATH to point to a generated ament file tree
            * Defaults ROS_HOME and ROS_LOG_DIR to $TEST_UNDECLARED_OUTPUTS_DIR (if set,
              otherwise to $TEST_TMPDIR, see https://bazel.build/reference/test-encyclopedia#initial-conditions)
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    _ros2_py_exec(py_test, name, srcs, main, set_up_ament, **kwargs)
