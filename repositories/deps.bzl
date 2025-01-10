"""Configures repo dependencies.
"""

load("@bazel_features//:deps.bzl", "bazel_features_deps")
load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load("@googletest//:googletest_deps.bzl", "googletest_deps")
load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

def ros2_deps():
    """ Sets up dependencies.
    """
    bazel_features_deps()
    bazel_skylib_workspace()
    rules_foreign_cc_dependencies()
    googletest_deps()
