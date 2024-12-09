load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def rust_setup_stage_1():
    maybe(
        http_archive,
        name = "rules_rust",
        sha256 = "b191375028448a69532046e901d8e6e627a6b00d58bec79027a05c1d3e090d00",
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.55.1/rules_rust-0.55.1.tar.gz",
    )

    maybe(
        http_archive,
        name = "rules_rust_bindgen",
        sha256 = "866c67cb176971cde5d935dc2c4c21e877718ca78f1fbc30b9699f41205f5815",
        urls = ["https://github.com/bazelbuild/rules_rust/releases/download/0.55.1/rules_rust_bindgen-0.55.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_rust.BUILD.bazel",
        sha256 = "9a57b561b1a68a7c70aed87584668249ccfa7caa23cc177378607daf8323d1a5",
        strip_prefix = "ros2_rust-31e4563e2b5a19f62f4841138927a215978bf01c",
        urls = ["https://github.com/ros2-rust/ros2_rust/archive/31e4563e2b5a19f62f4841138927a215978bf01c.zip"],
        patch_args = ["-p1"],
        patches = [
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rcl_bindings.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_generator.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_no_msg_vendoring.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_logging.patch",
        ],
    )
