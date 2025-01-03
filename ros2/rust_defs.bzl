""" Defines commonly used Rust macros.
"""

load(
    "@com_github_mvukov_rules_ros2//ros2:ament.bzl",
    "sh_launcher_rule",
    "split_kwargs",
)
load("@rules_rust//rust:defs.bzl", "rust_binary", "rust_test")

def ros2_rust_test(name, **kwargs):
    """ Defines a ROS 2 Rust test.

    Defaults ROS_HOME and ROS_LOG_DIR to $TEST_UNDECLARED_OUTPUTS_DIR (if set,
    otherwise to $TEST_TMPDIR, see https://bazel.build/reference/test-encyclopedia#initial-conditions)

    Please make sure that --sandbox_default_allow_network=false is set in .bazelrc.
    This ensures proper network isolation.

    Args:
        name: A unique target name.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    launcher_target_kwargs, test_kwargs = split_kwargs(**kwargs)

    target_impl = "{}_impl".format(name)
    rust_test(
        name = target_impl,
        tags = ["manual"],
        **test_kwargs
    )

    launcher = "{}_launch".format(name)
    sh_launcher_rule(
        name = launcher,
        template = "@com_github_mvukov_rules_ros2//ros2:launch.sh.tpl",
        substitutions = {
            "{entry_point}": "$(rootpath {})".format(target_impl),
        },
        tags = ["manual"],
        data = [target_impl],
        testonly = True,
    )

    native.sh_test(
        name = name,
        srcs = [launcher],
        data = [target_impl],
        **launcher_target_kwargs
    )
