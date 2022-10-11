load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def ros2_repositories():
    maybe(
        http_archive,
        name = "ros2_ament_cmake_ros",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ament_cmake_ros.BUILD.bazel",
        sha256sum = "6d7d8e4612e155953327d40a7c4d6c6c57ab02f6accfc21969bae679618a5560",
        strip_prefix = "ament_cmake_ros-0.9.2",
        url = "https://github.com/ros2/ament_cmake_ros/archive/refs/tags/0.9.2.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_common_interfaces",
        build_file = "@com_github_mvukov_rules_ros2//repositories:common_interfaces.BUILD.bazel",
        sha256sum = "8cfe9b00f0dc75e6e5e2fae8de0b2c84564ce43b94c68b227e317d4dcfe77073",
        strip_prefix = "common_interfaces-2.0.5",
        url = "https://github.com/ros2/common_interfaces/archive/refs/tags/2.0.5.tar.gz",
    )
