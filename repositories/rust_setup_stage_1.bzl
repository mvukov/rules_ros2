load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def rust_setup_stage_1():
    maybe(
        http_archive,
        name = "rules_rust",
        sha256 = "c9d5a39cb3e1ceacace98f19f09d61c4b55034bc34a565bdcf7cd80eb4c412af",
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.62.0/rules_rust-0.62.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "rules_rust_bindgen",
        sha256 = "c9d5a39cb3e1ceacace98f19f09d61c4b55034bc34a565bdcf7cd80eb4c412af",
        strip_prefix = "extensions/bindgen",
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.62.0/rules_rust-0.62.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_rust.BUILD.bazel",
        sha256 = "df8d5c118dd240094bac592bf249d5649a3d27b56a8623eba370070384c0e83c",
        strip_prefix = "ros2_rust-75cd88ef235094d1ad11326732b4fe447659e03c",
        urls = ["https://github.com/ros2-rust/ros2_rust/archive/75cd88ef235094d1ad11326732b4fe447659e03c.zip"],
        patch_args = ["-p1"],
        patches = [
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rclrs.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_generator.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_runtime.patch",
        ],
    )
