load("@com_github_mvukov_rules_ros2//repositories:repositories.bzl", "ros2_repositories")

def _non_module_deps_impl(mctx):
    ros2_repositories()

non_module_deps = module_extension(
    implementation = _non_module_deps_impl,
)
