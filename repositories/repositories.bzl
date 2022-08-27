"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

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
        name = "ros2_rcutils",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcutils.BUILD.bazel",
        sha256 = "ce4e0148c4d759ce93fda96d36e0009ef2f2974b3ca0a67e022ce8cc078cd43f",
        strip_prefix = "rcutils-1.1.3",
        urls = ["https://github.com/ros2/rcutils/archive/1.1.3.tar.gz"],
    )

    # maybe(
    #     native.new_local_repository,
    #     name = "ros2_rosidl",
    #     build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl.BUILD.bazel",
    #     path = "../ros2_foxy/src/ros2/rosidl",
    # )

    # foxy branch.
    maybe(
        new_git_repository,
        name = "ros2_rosidl",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl.BUILD.bazel",
        remote = "https://github.com/mvukov/rosidl.git",
        commit = "611c16181cfa832d2433e77c612bd75e303cca4e",
    )

    maybe(
        http_archive,
        name = "ros2_rmw",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw.BUILD.bazel",
        sha256 = "5e437ff6f6ff3e8b4b4bb4afc013efa7d4a5b28ace8686e94889d1f6a703fa53",
        strip_prefix = "rmw-1.0.3",
        urls = ["https://github.com/ros2/rmw/archive/1.0.3.tar.gz"],
    )

    # maybe(
    #     native.new_local_repository,
    #     name = "ros2_rcpputils",
    #     build_file = "@com_github_mvukov_rules_ros2//repositories:rcpputils.BUILD.bazel",
    #     path = "../ros2_foxy/src/ros2/rcpputils",
    # )

    # foxy branch.
    maybe(
        new_git_repository,
        name = "ros2_rcpputils",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcpputils.BUILD.bazel",
        remote = "https://github.com/mvukov/rcpputils.git",
        commit = "190fe233f9a46de715232fb07d195e5b43b21afe",
    )

    # maybe(
    #     native.new_local_repository,
    #     name = "ros2_rmw_implementation",
    #     build_file = "@com_github_mvukov_rules_ros2//repositories:rmw_implementation.BUILD.bazel",
    #     path = "../ros2_foxy/src/ros2/rmw_implementation",
    # )

    # foxy branch.
    maybe(
        new_git_repository,
        name = "ros2_rmw_implementation",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw_implementation.BUILD.bazel",
        remote = "https://github.com/mvukov/rmw_implementation.git",
        commit = "026462a1aac90352c67ba38b62652129144c821b",
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

    maybe(
        http_archive,
        name = "ros2_rclcpp",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rclcpp.BUILD.bazel",
        sha256 = "b231f234342abb7f7b749ff769257535fbccb90fa0982bc4c3e476eaf403f465",
        strip_prefix = "rclcpp-d12ed36e89fc3440acc82ef6563273de95d5a008",
        urls = ["https://github.com/ros2/rclcpp/archive/d12ed36e89fc3440acc82ef6563273de95d5a008.tar.gz"],
    )

    # maybe(
    #     native.new_local_repository,
    #     name = "ros2_rosidl_typesupport",
    #     build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_typesupport.BUILD.bazel",
    #     path = "../ros2_foxy/src/ros2/rosidl_typesupport",
    # )

    # foxy branch.
    maybe(
        new_git_repository,
        name = "ros2_rosidl_typesupport",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_typesupport.BUILD.bazel",
        remote = "https://github.com/mvukov/rosidl_typesupport.git",
        commit = "6a5e55ff8e1fbdd72a55060ec1ccedbef8f78f96",
    )

    maybe(
        http_archive,
        name = "ros2_libstatistics_collector",
        build_file = "@com_github_mvukov_rules_ros2//repositories:libstatistics_collector.BUILD.bazel",
        sha256 = "ff473044c51400cdd8ee73754e78723bc34401326e3fab257a70a031b3b56064",
        strip_prefix = "libstatistics_collector-562e0dd9e388ef4309b4630f2fc731d949235465",
        urls = ["https://github.com/ros-tooling/libstatistics_collector/archive/562e0dd9e388ef4309b4630f2fc731d949235465.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_common_interfaces",
        build_file = "@com_github_mvukov_rules_ros2//repositories:common_interfaces.BUILD.bazel",
        sha256 = "cb190749506d0612fcee1a8712da49d1509899c75d91035f4da83469c0746a9d",
        strip_prefix = "common_interfaces-2.0.4",
        urls = ["https://github.com/ros2/common_interfaces/archive/2.0.4.tar.gz"],
    )

    maybe(
        http_archive,
        name = "cyclonedds",
        build_file = "@com_github_mvukov_rules_ros2//repositories:cyclonedds.BUILD.bazel",
        sha256 = "b4488c641f914e8c51827dc50151edc1ffd915a53ed0f61c25f74d249267627d",
        strip_prefix = "cyclonedds-457a07132d394e6461ca23a5c27ddaefbb8ab868",
        urls = ["https://github.com/eclipse-cyclonedds/cyclonedds/archive/457a07132d394e6461ca23a5c27ddaefbb8ab868.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_rmw_dds_common",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw_dds_common.BUILD.bazel",
        sha256 = "34e46de0e2858af57d996b5d17fbfd76b58b1c37b64321d2c6bafaf4198d64db",
        strip_prefix = "rmw_dds_common-1.0.3",
        urls = ["https://github.com/ros2/rmw_dds_common/archive/1.0.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_rmw_cyclonedds",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rmw_cyclonedds.BUILD.bazel",
        sha256 = "5995d8ae3613126ee5b68db68d0c1c4a6caa8eec3fb0a269921a16fa1a810af6",
        strip_prefix = "rmw_cyclonedds-0.7.6",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:rmw_cyclonedds-fix-typesupport-conditions-bug.patch"],
        urls = ["https://github.com/ros2/rmw_cyclonedds/archive/0.7.6.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_unique_identifier_msgs",
        build_file = "@com_github_mvukov_rules_ros2//repositories:unique_identifier_msgs.BUILD.bazel",
        sha256 = "aa0f5a440cface1dd85cf05d97cadb812b2796973882d02a7e795ae70b64b9a0",
        strip_prefix = "unique_identifier_msgs-2.1.3",
        urls = ["https://github.com/ros2/unique_identifier_msgs/archive/2.1.3.tar.gz"],
    )

    maybe(
        native.new_local_repository,
        name = "ros2_rosidl_python",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_python.BUILD.bazel",
        path = "../ros2_foxy/src/ros2/rosidl_python",
    )

    maybe(
        native.new_local_repository,
        name = "ros2_rpyutils",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rpyutils.BUILD.bazel",
        path = "../ros2_foxy/src/ros2/rpyutils",
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
