"""Configures repo dependencies.
"""

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load("@rules_python//python:pip.bzl", "package_annotation", "pip_parse")
load("//repositories:python.bzl", "python_configuration")

ANNOTATIONS = {
    "numpy": package_annotation(
        additive_build_content = """\
cc_library(
    name = "headers",
    hdrs = glob(["site-packages/numpy/core/include/numpy/**/*.h"]),
    includes = ["site-packages/numpy/core/include"],
    deps = ["@local_config_python//:headers"],
)
""",
    ),
}

def ros2_deps(python_interpreter, python_requirements_lock):  # buildifier: disable=unnamed-macro
    """ Sets up dependencies.

    Args:
        python_interpreter: A Python interpreter.
        python_requirements_lock: A requirements lock file.
    """
    python_configuration(
        name = "local_config_python",
        python_interpreter = python_interpreter,
    )

    native.register_toolchains("@local_config_python//:py3_toolchain")

    if not native.existing_rule("rules_ros2_pip_deps"):
        pip_parse(
            name = "rules_ros2_pip_deps",
            python_interpreter = python_interpreter,
            requirements_lock = python_requirements_lock,
            annotations = ANNOTATIONS,
        )

    bazel_skylib_workspace()
