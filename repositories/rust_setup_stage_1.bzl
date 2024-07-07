load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def rust_setup_stage_1():
    maybe(
        http_archive,
        name = "rules_rust",
        sha256 = "17c53bf800b932f32d3ca19d2cb9e8ad533ce1c0d729f0d183077bfddab7ad46",
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:rules_rust_fix_collect_deps.patch"],
        url = "https://github.com/bazelbuild/rules_rust/releases/download/0.46.0/rules_rust-v0.46.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "ros2_rust",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2_rust.BUILD.bazel",
        sha256 = "ec3ebfb177f0ecfc2c6ecb47f2fffe701f6ebeb2b29ab482b57b53dd2c260da2",
        strip_prefix = "ros2_rust-2e746d6e0d2cbeed811132310c3a8dc2da4f9975",
        urls = ["https://github.com/ros2-rust/ros2_rust/archive/2e746d6e0d2cbeed811132310c3a8dc2da4f9975.zip"],
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
