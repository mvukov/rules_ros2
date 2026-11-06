"""Handles imports of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load(
    "@com_github_mvukov_rules_ros2//repositories:ros2_repositories_impl.bzl",
    "ros2_repositories_impl",
)

def ros2_repositories():
    """Import ROS 2 repositories."""

    http_archive(
        name = "ros2",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2.BUILD.bazel",
        sha256 = "7e55ac684cb1647e463f2bebcaa55bf14164c38d2fa011c0c4e44bafe8140f46",
        strip_prefix = "ros2-release-jazzy-20250820",
        urls = ["https://github.com/ros2/ros2/archive/refs/tags/release-jazzy-20250820.tar.gz"],
    )

    ros2_repositories_impl()

    # ros2-devel branch
    http_archive(
        name = "ros2_gps_umd",
        build_file = "@com_github_mvukov_rules_ros2//repositories:gps_umd.BUILD.bazel",
        sha256 = "64a96f93053d0d59e8fcccceab5408a7d666dd813d4c12df139ef24d916f49ab",
        strip_prefix = "gps_umd-fc782811804fafb12ee479a48a2aa2e9ee942e2d",
        urls = ["https://github.com/swri-robotics/gps_umd/archive/fc782811804fafb12ee479a48a2aa2e9ee942e2d.tar.gz"],
    )

    http_archive(
        name = "foxglove_bridge",
        build_file = "@com_github_mvukov_rules_ros2//repositories:foxglove_bridge.BUILD.bazel",
        sha256 = "9548f6a53794cfcd6dcae570e6e82aa2c7670d177269ecf612838655d7ba7dcc",
        strip_prefix = "ros-foxglove-bridge-0.7.9",
        urls = ["https://github.com/foxglove/ros-foxglove-bridge/archive/refs/tags/0.7.9.tar.gz"],
    )

    http_archive(
        name = "ros2_xacro",
        build_file = "@com_github_mvukov_rules_ros2//repositories:xacro.BUILD.bazel",
        sha256 = "f9d94956574015427e59011d4ee113b206e9c10a27a0c01d4b08ee4268d76741",
        strip_prefix = "xacro-2.1.1",
        urls = ["https://github.com/ros/xacro/archive/refs/tags/2.1.1.tar.gz"],
    )

    # Version copied from https://github.com/ros2/orocos_kdl_vendor/blob/0.5.2/orocos_kdl_vendor/CMakeLists.txt#L19.
    http_archive(
        name = "orocos_kdl",
        build_file = "@com_github_mvukov_rules_ros2//repositories:orocos_kdl.BUILD.bazel",
        sha256 = "8acf1dc9c2f7ae37d747f16ce1e8e3cb40eb9187b8950aa2871f944b55520816",
        strip_prefix = "orocos_kinematics_dynamics-ce4bcb65a050615b6d7f21bc60fbb2656515791b",
        urls = ["https://github.com/orocos/orocos_kinematics_dynamics/archive/ce4bcb65a050615b6d7f21bc60fbb2656515791b.tar.gz"],
    )

    http_archive(
        name = "ros2_urdf",
        build_file = "@com_github_mvukov_rules_ros2//repositories:urdf.BUILD.bazel",
        sha256 = "7bd84e41f20c50719de2eeff1d8be58dbc6cafcd93ea88dc58072d5dcf8f8ca5",
        strip_prefix = "urdf-2.10.1",
        urls = ["https://github.com/ros2/urdf/archive/refs/tags/2.10.1.tar.gz"],
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:urdf_plugin_loader_fix.patch"],
    )

    http_archive(
        name = "ros2_diagnostics",
        build_file = "@com_github_mvukov_rules_ros2//repositories:diagnostics.BUILD.bazel",
        sha256 = "bf275beae52d3f75e7a8cb7568723651fc8ab2e6e247e261052107643a71228d",
        strip_prefix = "diagnostics-4.5.7",
        urls = ["https://github.com/ros/diagnostics/archive/refs/tags/4.5.7.tar.gz"],
    )

    http_archive(
        name = "console_bridge",
        build_file = "@com_github_mvukov_rules_ros2//repositories:console_bridge.BUILD.bazel",
        sha256 = "303a619c01a9e14a3c82eb9762b8a428ef5311a6d46353872ab9a904358be4a4",
        strip_prefix = "console_bridge-1.0.2",
        urls = ["https://github.com/ros/console_bridge/archive/1.0.2.tar.gz"],
    )

    http_archive(
        name = "mcap",
        build_file = "@com_github_mvukov_rules_ros2//repositories:mcap.BUILD.bazel",
        sha256 = "69bcd33e1590201ea180e9e6ba8a22f3e8e7e2283e9ebcbff6a2c7134d8341db",
        strip_prefix = "mcap-releases-cpp-v1.4.1/cpp/mcap",
        urls = ["https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v1.4.1.tar.gz"],
    )

    # NOTE: Use the jazzy branch.
    http_archive(
        name = "ros2_rcl_logging_syslog",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcl_logging_syslog.BUILD.bazel",
        sha256 = "748a39586307ce375999f0b6a861e5b36c992d6aaeffa2641f2213171f37affb",
        strip_prefix = "rcl_logging_syslog-1b8165639d34f6321f725de5b4d72effb2d15314",
        urls = ["https://github.com/fujitatomoya/rcl_logging_syslog/archive/1b8165639d34f6321f725de5b4d72effb2d15314.zip"],
    )

    http_archive(
        name = "ros2_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_rust.BUILD.bazel",
        sha256 = "8ba24f9809dff77306e5866ebef1575d78355dc63202855f388895ee26c70097",
        strip_prefix = "ros2_rust-0.6.0",
        url = "https://github.com/ros2-rust/ros2_rust/archive/refs/tags/v0.6.0.tar.gz",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rclrs.patch"],
    )

    http_archive(
        name = "ros2_rosidl_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_rust.BUILD.bazel",
        sha256 = "e572a55e8e2e8cb9e3dff7e692a9b59562b0b171b79a26229d5dad576a265199",
        strip_prefix = "rosidl_rust-0.5.0",
        url = "https://github.com/ros2-rust/rosidl_rust/archive/refs/tags/0.5.0.tar.gz",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_generator.patch"],
    )

    http_archive(
        name = "ros2_rosidl_runtime_rs",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_runtime_rs.BUILD.bazel",
        sha256 = "de7254fcbca2f8f77ac196d9419798d6856bb0fbea4f586907bf6a81396692c6",
        strip_prefix = "rosidl_runtime_rs-0.5.0",
        url = "https://github.com/ros2-rust/rosidl_runtime_rs/archive/refs/tags/v0.5.0.tar.gz",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_runtime.patch"],
    )
