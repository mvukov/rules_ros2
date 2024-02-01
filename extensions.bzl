load("@com_github_mvukov_rules_ros2//repositories:repositories.bzl", "ros2_repositories", "rules_ros2_workspace_and_bazelmod_common_deps")

def _non_module_deps_impl(mctx):
    rules_ros2_workspace_and_bazelmod_common_deps()
    ros2_repositories()

non_module_deps = module_extension(
    implementation = _non_module_deps_impl,
)
