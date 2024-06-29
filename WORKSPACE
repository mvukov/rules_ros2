workspace(name = "com_github_mvukov_rules_ros2")

load("//repositories:repositories.bzl", "ros2_repositories", "ros2_workspace_repositories")

ros2_workspace_repositories()

ros2_repositories()

load("//repositories:deps.bzl", "ros2_deps")

ros2_deps()

load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_multi_toolchains")

py_repositories()

python_register_multi_toolchains(
    name = "rules_ros2_pythons",
    default_version = "3.10",
    python_versions = ["3.10"],
)

load("@rules_python//python:pip.bzl", "pip_parse")
load("@rules_ros2_pythons//3.10:defs.bzl", python_interpreter_target = "interpreter")

pip_parse(
    name = "rules_ros2_pip_deps",
    python_interpreter_target = python_interpreter_target,
    requirements_lock = "@com_github_mvukov_rules_ros2//:requirements_lock.txt",
)

load(
    "@rules_ros2_pip_deps//:requirements.bzl",
    install_rules_ros2_pip_deps = "install_deps",
)

install_rules_ros2_pip_deps()

# Below are internal deps.

pip_parse(
    name = "rules_ros2_resolver_deps",
    python_interpreter_target = python_interpreter_target,
    requirements_lock = "//repositories/private:resolver_requirements_lock.txt",
)

load(
    "@rules_ros2_resolver_deps//:requirements.bzl",
    install_rules_ros2_resolver_deps = "install_deps",
)

install_rules_ros2_resolver_deps()
