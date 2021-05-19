"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def ros2_repositories():
    """Imports external/third-party repositories.

    At the moment ROS2 package versions target ROS2 Foxy.
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
        urls = ["https://github.com/ros2/rcutils/archive/1.1.3.tar.gz"],
    )

    maybe(
        native.new_local_repository,
        name = "ros2_rosidl",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl.BUILD.bazel",
        path = "../ros2_foxy/src/ros2/rosidl",
        # sha256 = "d839ccfa148ca27686759921300d67b88366d5fc320c3c8b483c04362bd89a2c",
        # strip_prefix = "rosidl-1.2.1",
        # urls = ["https://github.com/ros2/rosidl/archive/1.2.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_rmw",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw.BUILD.bazel",
        sha256 = "5e437ff6f6ff3e8b4b4bb4afc013efa7d4a5b28ace8686e94889d1f6a703fa53",
        strip_prefix = "rmw-1.0.3",
        urls = ["https://github.com/ros2/rmw/archive/1.0.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_rcpputils",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcpputils.BUILD.bazel",
        sha256 = "20b9863c516a48b20ff0dadd02bf17a3d1c42c3c47b07f78e118cf9c3cd000c9",
        strip_prefix = "rcpputils-1.3.1",
        urls = ["https://github.com/ros2/rcpputils/archive/1.3.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_rmw_implementation",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw_implementation.BUILD.bazel",
        sha256 = "b56555fc55bb5897d82b3c5585ef6c70218ec14fb49faee5a9c5526d6fc1fec0",
        strip_prefix = "rmw_implementation-1.0.2",
        urls = ["https://github.com/ros2/rmw_implementation/archive/1.0.2.tar.gz"],
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
        name = "ros2_rcl_logging",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcl_logging.BUILD.bazel",
        sha256 = "c7a4a8f22b77269fe99ad43c1e075bf2a3f42513bfb2531fd9e5fe9f146e6e63",
        strip_prefix = "rcl_logging-1.1.0",
        urls = ["https://github.com/ros2/rcl_logging/archive/1.1.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "libyaml",
        build_file = "@com_github_mvukov_rules_ros2//repositories:libyaml.BUILD.bazel",
        sha256 = "2c103fc473e904c6fe7580277f1fa16b6d716e54d5e3f32a8913c4850ae03b3f",
        strip_prefix = "libyaml-acd6f6f014c25e46363e718381e0b35205df2d83",
        urls = ["https://github.com/yaml/libyaml/archive/acd6f6f014c25e46363e718381e0b35205df2d83.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_rcl",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcl.BUILD.bazel",
        sha256 = "f56b714081c866af2e83133841fabc301f3c91190d1abd3ec2a2835712886080",
        strip_prefix = "rcl-1.1.11",
        urls = ["https://github.com/ros2/rcl/archive/1.1.11.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_tracing",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_tracing.BUILD.bazel",
        sha256 = "e8ff2628f24a2e31b8c884780898c074841a9c0dc92f7d00e079eb2cd0ba6d6d",
        strip_prefix = "ros2_tracing-6535577457eb8fb8ec23373b0829590b27ff50c0",
        urls = ["https://gitlab.com/ros-tracing/ros2_tracing/-/archive/6535577457eb8fb8ec23373b0829590b27ff50c0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_rcl_interfaces",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcl_interfaces.BUILD.bazel",
        sha256 = "0579862f4031dbefccec61a012f7e559f3c3a08cfc2ed0e1c72015eff47b7be6",
        strip_prefix = "rcl_interfaces-ee04e046132be9d9fdcdaeff8dd54a5d0d9aa022",
        urls = ["https://github.com/ros2/rcl_interfaces/archive/ee04e046132be9d9fdcdaeff8dd54a5d0d9aa022.tar.gz"],
    )

