load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def rust_setup_stage_1():
    maybe(
        http_archive,
        name = "rules_rust",
        sha256 = "c8aa806cf6066679ac23463241ee80ad692265dad0465f51111cbbe30b890352",
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.68.1/rules_rust-0.68.1.tar.gz",
    )

    ros2_rust_repositories()

def ros2_rust_repositories():
    maybe(
        http_archive,
        name = "ros2_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_rust.BUILD.bazel",
        sha256 = "8ba24f9809dff77306e5866ebef1575d78355dc63202855f388895ee26c70097",
        strip_prefix = "ros2_rust-0.6.0",
        url = "https://github.com/ros2-rust/ros2_rust/archive/refs/tags/v0.6.0.tar.gz",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rclrs.patch"],
    )

    maybe(
        http_archive,
        name = "ros2_rosidl_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_rust.BUILD.bazel",
        sha256 = "64f145210ac165717c050232e0867cc027f59746b8541e87502b7fa71eaa3953",
        strip_prefix = "rosidl_rust-0.4.9",
        url = "https://github.com/ros2-rust/rosidl_rust/archive/refs/tags/0.4.9.tar.gz",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_generator.patch"],
    )

    maybe(
        http_archive,
        name = "ros2_rosidl_runtime_rs",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rosidl_runtime_rs.BUILD.bazel",
        sha256 = "de7254fcbca2f8f77ac196d9419798d6856bb0fbea4f586907bf6a81396692c6",
        strip_prefix = "rosidl_runtime_rs-0.5.0",
        url = "https://github.com/ros2-rust/rosidl_runtime_rs/archive/refs/tags/v0.5.0.tar.gz",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_runtime.patch"],
    )