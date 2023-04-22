""" Defines commonly used Python macros.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "sh_launcher")
load("@com_github_mvukov_rules_ros2//third_party:symlink.bzl", "symlink")
load("@rules_python//python:defs.bzl", "py_binary", "py_test")

def _ros2_py_exec(target, name, srcs, main, set_up_ament = False, set_up_ros_home = False, **kwargs):
    if not (set_up_ament or set_up_ros_home):
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
            "{{set_up_ros_home}}": "set_up_ros_home" if set_up_ros_home else "",
            "{{set_up_ament}}": "set_up_ament" if set_up_ament else "",
            "{{entry_point}}": "$(rootpath {})".format(target_impl_symlink),
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
        set_up_ament: If true, sets up ament file tree for the binary target.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    if "set_up_ros_home" in kwargs:
        fail("set_up_ros_home only makes sense for test targets.")
    _ros2_py_exec(py_binary, name, srcs, main, set_up_ament, **kwargs)

def ros2_py_test(name, srcs, main, set_up_ament = False, set_up_ros_home = True, **kwargs):
    """ Defines a ROS 2 Python test.

    Args:
        name: A unique target name.
        srcs: List of source files.
        main: Source file to use as entrypoint.
        set_up_ament: If true, sets up ament file tree for the test target.
        set_up_ros_home: If true, sets up ROS_HOME for the test target.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    _ros2_py_exec(py_test, name, srcs, main, set_up_ament, set_up_ros_home, **kwargs)
