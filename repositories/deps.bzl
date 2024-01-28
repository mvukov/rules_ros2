"""Configures repo dependencies.
"""

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

def ros2_deps():
    """ Sets up dependencies.
    """
    bazel_skylib_workspace()
    rules_foreign_cc_dependencies()
