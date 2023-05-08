""" Defines commonly used Python macros.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "sh_launcher")
load("@com_github_mvukov_rules_ros2//third_party:symlink.bzl", "symlink")
load("@rules_python//python:defs.bzl", "py_binary", "py_test")

def _ros2_py_exec(target, name, srcs, main, set_up_ament, **kwargs):
    if set_up_ament == False:
        target(name = name, srcs = srcs, main = main, **kwargs)
        return

    target_impl = name + "_impl"
    tags = kwargs.pop("tags", [])
    visibility = kwargs.pop("visibility", None)
    size = kwargs.pop("size", None)
    target(name = target_impl, srcs = srcs, main = main, tags = ["manual"], **kwargs)

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
        size = size,
        srcs = [launcher],
        data = [target_impl_symlink],
        tags = tags,
        visibility = visibility,
    )

def ros2_py_binary(name, srcs, main, set_up_ament = False, **kwargs):
    """ Defines a ROS 2 Python binary.

    Args:
        name: A unique target name.
        srcs: List of source files.
        main: Source file to use as entrypoint.
        set_up_ament: If true, wrap the binary target to set up ament file tree and
            missing ROS_HOME/ROS_LOG_DIR env vars.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    _ros2_py_exec(py_binary, name, srcs, main, set_up_ament, **kwargs)

def ros2_py_test(name, srcs, main, set_up_ament = True, **kwargs):
    """ Defines a ROS 2 Python test.

    Args:
        name: A unique target name.
        srcs: List of source files.
        main: Source file to use as entrypoint.
        set_up_ament: If true, wrap the test target to set up ament file tree and
            missing ROS_HOME/ROS_LOG_DIR env vars.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    _ros2_py_exec(py_test, name, srcs, main, set_up_ament, **kwargs)
