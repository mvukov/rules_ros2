"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
load(
    "@com_github_mvukov_rules_ros2//repositories:ros2_repos.bzl",
    "ros2_repos",
)

def ros2_repositories():
    """Imports external/third-party repositories.

    At the moment ROS2 package versions target ROS2 Foxy.
    """
    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "c03246c11efd49266e8e41e12931090b613e12a59e6f55ba2efd29a7cb8b4258",
        strip_prefix = "rules_python-0.11.0",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.11.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz"],
        sha256 = "1c531376ac7e5a180e0237938a2536de0c54d93f5c278634818e0efc952dd56c",
    )

    maybe(
        http_archive,
        name = "fmt",
        build_file = "@com_github_mvukov_rules_ros2//repositories:fmt.BUILD.bazel",
        sha256 = "5cae7072042b3043e12d53d50ef404bbb76949dad1de368d7f993a15c8c05ecc",
        strip_prefix = "fmt-7.1.3",
        url = "https://github.com/fmtlib/fmt/archive/7.1.3.tar.gz",
    )

    maybe(
        http_archive,
        name = "spdlog",
        build_file = "@com_github_mvukov_rules_ros2//repositories:spdlog.BUILD.bazel",
        sha256 = "944d0bd7c763ac721398dca2bb0f3b5ed16f67cef36810ede5061f35a543b4b8",
        strip_prefix = "spdlog-1.8.5",
        url = "https://github.com/gabime/spdlog/archive/v1.8.5.tar.gz",
    )

    maybe(
        http_archive,
        name = "libyaml",
        build_file = "@com_github_mvukov_rules_ros2//repositories:libyaml.BUILD.bazel",
        sha256 = "2c103fc473e904c6fe7580277f1fa16b6d716e54d5e3f32a8913c4850ae03b3f",
        strip_prefix = "libyaml-acd6f6f014c25e46363e718381e0b35205df2d83",
        urls = ["https://github.com/yaml/libyaml/archive/acd6f6f014c25e46363e718381e0b35205df2d83.tar.gz"],
    )

    # ros2-devel branch
    maybe(
        http_archive,
        name = "ros2_gps_umd",
        build_file = "@com_github_mvukov_rules_ros2//repositories:gps_umd.BUILD.bazel",
        sha256 = "64a96f93053d0d59e8fcccceab5408a7d666dd813d4c12df139ef24d916f49ab",
        strip_prefix = "gps_umd-fc782811804fafb12ee479a48a2aa2e9ee942e2d",
        urls = ["https://github.com/swri-robotics/gps_umd/archive/fc782811804fafb12ee479a48a2aa2e9ee942e2d.tar.gz"],
    )

    # maybe(
    #     native.new_local_repository,
    #     name = "ros2_launch",
    #     build_file = "@com_github_mvukov_rules_ros2//repositories:launch.BUILD.bazel",
    #     path = "../ros2_foxy/src/ros2/launch",
    # )

    # feature/bazel branch
    maybe(
        new_git_repository,
        name = "ros2_launch",
        build_file = "@com_github_mvukov_rules_ros2//repositories:launch.BUILD.bazel",
        remote = "https://github.com/mvukov/launch.git",
        commit = "e0268d0d6b60be01cfa639d2c0f77306eba51539",
    )

    # tied to ros2_launch!
    maybe(
        http_archive,
        name = "ros2_launch_ros",
        build_file = "@com_github_mvukov_rules_ros2//repositories:launch_ros.BUILD.bazel",
        sha256 = "752949dd63a63db0ab8ef0aca3b03a1e910ca327b958cc01cdcb487c161b2ed4",
        strip_prefix = "launch_ros-0.11.2",
        urls = ["https://github.com/ros2/launch_ros/archive/0.11.2.tar.gz"],
    )

    # maybe(
    #     native.new_local_repository,
    #     name = "ros2cli",
    #     build_file = "@com_github_mvukov_rules_ros2//repositories:ros2cli.BUILD.bazel",
    #     path = "../ros2_foxy/src/ros2/ros2cli",
    # )

    # feature/bazel branch
    maybe(
        new_git_repository,
        name = "ros2cli",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2cli.BUILD.bazel",
        remote = "https://github.com/mvukov/ros2cli.git",
        commit = "087658b4ab3adedb1e8355af8f4d2c2e52d13c9b",
    )

    maybe(
        http_archive,
        name = "ros2",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2.BUILD.bazel",
        sha256 = "86039b6c4cd7953edaaf1ef96eafd9770dd7cd2750379d2507b62ca195bbe76e",
        strip_prefix = "ros2-release-foxy-20220928",
        urls = ["https://github.com/ros2/ros2/archive/refs/tags/release-foxy-20220928.tar.gz"],
    )

    ros2_repos()
