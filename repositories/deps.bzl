"""Configures repo dependencies.
"""

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load("@rules_python//python:pip.bzl", "pip_parse")

def ros2_deps(python_interpreter, python_requirements_lock):
    if not native.existing_rule("rules_ros2_pip_deps"):
        pip_parse(
            name = "rules_ros2_pip_deps",
            python_interpreter = python_interpreter,
            requirements_lock = python_requirements_lock,
        )

    bazel_skylib_workspace()
