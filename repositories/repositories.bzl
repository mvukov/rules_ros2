"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def ros2_repositories():
    """Imports external/third-party repositories.
    """
    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "778197e26c5fbeb07ac2a2c5ae405b30f6cb7ad1f5510ea6fdac03bded96cc6f",
        urls = ["https://github.com/bazelbuild/rules_python/releases/download/0.2.0/rules_python-0.2.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz"],
        sha256 = "1c531376ac7e5a180e0237938a2536de0c54d93f5c278634818e0efc952dd56c",
    )

    maybe(
        http_archive,
        name = "ros2_rcutils",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcutils.BUILD.bazel",
        sha256 = "ce4e0148c4d759ce93fda96d36e0009ef2f2974b3ca0a67e022ce8cc078cd43f",
        strip_prefix = "rcutils-1.1.3",
        urls = ["https://github.com/ros2/rcutils/archive/refs/tags/1.1.3.tar.gz"],
    )
