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
        sha256 = "3dc4d57353ceb9d71f447c21a64c2bae18cf6312ef473a1de82edc53ddcf70b7",
        strip_prefix = "ros2_rust-dba36cc0b7670dc6bdf0a57b366f1e7ef132d933",
        urls = ["https://github.com/ros2-rust/ros2_rust/archive/dba36cc0b7670dc6bdf0a57b366f1e7ef132d933.zip"],
        patch_args = ["-p1"],
        patches = [
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rclrs.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_generator.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_runtime.patch",
        ],
    )
