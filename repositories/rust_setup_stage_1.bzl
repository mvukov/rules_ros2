load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def rust_setup_stage_1():
    maybe(
        http_archive,
        name = "rules_rust",
        sha256 = "dd006b77221d59e4d141207c0e7adf11b1fb60d1440b8fca03bf925617932a60",
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.49.3/rules_rust-v0.49.3.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_rust.BUILD.bazel",
        sha256 = "b5ce6dc04e2bced77cf90ac346da46e9f6a1c04c400fb5589cc8228cddffdea3",
        strip_prefix = "ros2_rust-e485b1c90893bedab32ee9e54ff5e47c4f6bd004",
        urls = ["https://github.com/ros2-rust/ros2_rust/archive/e485b1c90893bedab32ee9e54ff5e47c4f6bd004.zip"],
        patch_args = ["-p1"],
        patches = [
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rcl_bindings.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_fix_rosidl_generator.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:ros2_rust_no_msg_vendoring.patch",
        ],
    )

    # Only needed for CI, see https://github.com/bazelbuild/rules_rust/pull/2698.
    maybe(
        http_archive,
        name = "com_github_mvukov_rules_ros2_cargo_bazel",
        build_file_content = """
exports_files([
    "cargo-bazel",
])
""",
        sha256 = "6d6d68b898b38cb58d3c29e2e54d5795e2f652ebcac1ecf3bb0ac99fc86480e7",
        url = "https://github.com/mvukov/rules_rust/releases/download/0.46.0-mvukov-cargo-bazel/cargo-bazel.zip",
    )
