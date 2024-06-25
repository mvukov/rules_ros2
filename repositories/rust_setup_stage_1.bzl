load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def rust_setup_stage_1():
    maybe(
        http_archive,
        name = "rules_rust",
        sha256 = "4046f362cafd010df26b9a427d2bb51e4ea5b3f5dc5e1b7645cdc88cf7748ba2",
        strip_prefix = "rules_rust-33f93ace380f6bf26b0f4473ffae736360903422",
        patch_args = ["-p1"],
        patches = [
            "@com_github_mvukov_rules_ros2//repositories/patches:rules_rust_fix_collect_deps.patch",
            "@com_github_mvukov_rules_ros2//repositories/patches:rules_rust_fix_lockfile_bin_version.patch",
        ],
        urls = ["https://github.com/bazelbuild/rules_rust/archive/33f93ace380f6bf26b0f4473ffae736360903422.zip"],
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
