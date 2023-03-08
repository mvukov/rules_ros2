""" Defines commonly used Python macros.
"""

load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "sh_launcher")
load("@com_github_mvukov_rules_ros2//third_party:symlink.bzl", "symlink")
load("@rules_python//python:defs.bzl", "py_test")

def ros2_py_test(name, set_up_ament = False, **kwargs):
    """ Defines a ROS 2 Python test.

    Args:
        name: A unique target name.
        set_up_ament: If true, sets up ament file tree for the test target.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    if set_up_ament == False:
        py_test(name = name, **kwargs)
        return

    test_target = name + "_impl"
    tags = kwargs.pop("tags", None)
    py_test(name = test_target, tags = ["manual"], **kwargs)

    test_target_symlink = test_target + "_symlink"
    symlink(
        name = test_target_symlink,
        executable = test_target,
        testonly = True,
        tags = ["manual"],
    )

    launcher = "{}_launch".format(name)
    sh_launcher(
        launcher,
        deps = [test_target],
        template = "@com_github_mvukov_rules_ros2//ros2:test.sh.tpl",
        substitutions = {
            "{entry_point}": "$(rootpath {})".format(test_target_symlink),
        },
        tags = ["manual"],
        data = [test_target_symlink],
        testonly = True,
    )

    native.sh_test(
        name = name,
        srcs = [launcher],
        data = [test_target_symlink],
        tags = tags,
    )
