ROS 2 repositories are defined using an official release from https://github.com/ros2/ros2.
Such a release is captured in `repositories/repositories.bzl` file,
see `http_archive` named `ros2`.
The corresponding `ros2.BUILD.bazel` defines a simple filegroup with a YAML
file `ros2.repos` that stores info about versioned ROS 2 packages.

The contents from `ros2.repos` is mapped to Bazel repos using
`repositories/ros2_repo_mappings.yaml`. In that file, each ROS 2 repo has
a corresponding dictionary for generation of Bazel `http_archive` definitions.
The auto-generated definitions are stored in `repositories/ros2_repositories_impl.bzl` file.
To update the ROS 2 package definitions, run:

```bash
bazel run //repositories/private:resolver
```

The whole idea is to avoid updating versions in the `_impl.bzl` file by hand, but
to run the resolver binary instead. If you want to add extra info
for a ROS 2 repo that's already mapped or add another ROS 2 repo,
please update `repositories/ros2_repo_mappings.yaml`. If you changed the config
yaml file, please rerun the resolver app as shown above.

Repositories that are not part of `ros2.repos` (e.g. `osrf_pycommon`, which
was dropped from `ros2.repos` in Lyrical) are pinned manually in
`repositories/repositories.bzl`. When you move a repository from the mappings
yaml file to a manual pin, first remove its entry from
`repositories/ros2_repositories_impl.bzl` by hand: otherwise the module
extension defines the repository twice and the resolver can't run.

Naturally, what if you want to override an auto-generated definition?
All definitions are exposed through the `non_module_deps` module extension, so
in your root `MODULE.bazel` you can replace any of them with
[`override_repo`](https://bazel.build/rules/lib/globals/module#override_repo):

```starlark
http_archive = use_repo_rule("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "my_ros2_rclcpp",
    # ...
)

rules_ros2_non_module_deps = use_extension("@com_github_mvukov_rules_ros2//ros2:extensions.bzl", "non_module_deps")
override_repo(rules_ros2_non_module_deps, ros2_rclcpp = "my_ros2_rclcpp")
```
