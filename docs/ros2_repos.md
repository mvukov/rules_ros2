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

Naturally, what if you want to override an auto-generated definition?
Since all definitions use `maybe` Bazel macro, it is sufficient to define an
`http_archive` repo with the same name _before_ the auto-generated definition.
