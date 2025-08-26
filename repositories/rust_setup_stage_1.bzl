load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def rust_setup_stage_1():
    maybe(
        http_archive,
        name = "rules_rust",
        sha256 = "c38b622f26f35c34738100e26d1793ff252897381546467b22d89c9d4d8bfd50",
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.63.0/rules_rust-0.63.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "rules_rust_bindgen",
        sha256 = "c38b622f26f35c34738100e26d1793ff252897381546467b22d89c9d4d8bfd50",
        strip_prefix = "extensions/bindgen",
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.63.0/rules_rust-0.63.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_rust.BUILD.bazel",
        sha256 = "b0aa9c2a890968946dd29d2368fcba542cadd8399b5a0460a3559888b272209b",
        strip_prefix = "ros2_rust-v0.5.1",
        urls = ["https://github.com/ros2-rust/ros2_rust/archive/v0.5.1.tar.gz"],
        patch_args = ["-p1"],
        patches = [
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rclrs.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_generator.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_runtime.patch",
        ],
    )
