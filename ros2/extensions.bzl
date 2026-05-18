load("//repositories:clang_configure.bzl", "clang_configure")
load("//repositories:repositories.bzl", "proto2ros_repository", "ros2_repositories")
load("//repositories:rust_setup_stage_1.bzl", "ros2_rust_repositories")

def _non_module_deps_impl(mctx):
    ros2_repositories()
    proto2ros_repository()
    clang_configure(name = "rules_ros2_config_clang")
    ros2_rust_repositories()

non_module_deps = module_extension(
    implementation = _non_module_deps_impl,
)
