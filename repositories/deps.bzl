"""Configures repo dependencies.
"""

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")
load("@rules_python//python:pip.bzl", "package_annotation")

PIP_ANNOTATIONS = {
    "numpy": package_annotation(
        additive_build_content = """\
cc_library(
    name = "headers",
    hdrs = glob(["site-packages/numpy/core/include/numpy/**/*.h"]),
    includes = ["site-packages/numpy/core/include"],
    deps = ["@rules_ros2_python//:python_headers"],
)
""",
    ),
}

def ros2_deps():
    """ Sets up dependencies.
    """
    bazel_skylib_workspace()
    rules_foreign_cc_dependencies()
