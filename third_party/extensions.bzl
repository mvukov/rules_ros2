"""
This file contains the extensions for loading all the non-repo modules
"""

load("//repositories:repositories.bzl", "ros2_repositories")
load("//repositories:ros2_repositories_impl.bzl", "ros2_repositories_impl")

def _non_module_dependencies_impl(_ctx):
    ros2_repositories()
    ros2_repositories_impl()

non_module_dependencies = module_extension(
    implementation = _non_module_dependencies_impl,
)
