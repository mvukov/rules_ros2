load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def rust_setup_stage_1():
    maybe(
        http_archive,
        name = "rules_rust",
        sha256 = "09e17b47c0150465631aa319f2742760a43ededab2e9c012f91d0ae2eff02268",
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.59.2/rules_rust-0.59.2.tar.gz",
    )

    maybe(
        http_archive,
        name = "rules_rust_bindgen",
        sha256 = "09e17b47c0150465631aa319f2742760a43ededab2e9c012f91d0ae2eff02268",
        strip_prefix = "extensions/bindgen",
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.59.2/rules_rust-0.59.2.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_rust.BUILD.bazel",
        sha256 = "b5a8c51917535f6afdfddf426fc68c609cc9d664f8c3a36de573c98b51b12849",
        strip_prefix = "ros2_rust-d62583d2c9faab2f0342e1f803f8088b3627d148",
        urls = ["https://github.com/ros2-rust/ros2_rust/archive/d62583d2c9faab2f0342e1f803f8088b3627d148.zip"],
        patch_args = ["-p1"],
        patches = [
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rclrs.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_generator.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_runtime.patch",
        ],
    )
