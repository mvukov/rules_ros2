workspace(name = "com_github_mvukov_rules_ros2")

load("//repositories:repositories.bzl", "ros2_repositories", "ros2_workspace_repositories")

ros2_workspace_repositories()

ros2_repositories()

load("//repositories:deps.bzl", "ros2_deps")

ros2_deps()

load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")

py_repositories()

python_register_toolchains(
    name = "rules_ros2_python",
    python_version = "3.10",
)

load("@rules_python//python:pip.bzl", "pip_parse")

pip_parse(
    name = "rules_ros2_pip_deps",
    python_interpreter_target = "@rules_ros2_python_host//:python",
    requirements_lock = "@com_github_mvukov_rules_ros2//:requirements_lock.txt",
)

load(
    "@rules_ros2_pip_deps//:requirements.bzl",
    install_rules_ros2_pip_deps = "install_deps",
)

install_rules_ros2_pip_deps()

# Below is an optional setup for Rust support for ROS 2.

load("//repositories:rust_setup_stage_1.bzl", "rust_setup_stage_1")

rust_setup_stage_1()

load("//repositories:rust_setup_stage_2.bzl", "rust_setup_stage_2")

rust_setup_stage_2()

load("//repositories:rust_setup_stage_3.bzl", "rust_setup_stage_3")

rust_setup_stage_3()

load("//repositories:rust_setup_stage_4.bzl", "rust_setup_stage_4")

rust_setup_stage_4()

# Below are internal deps.

pip_parse(
    name = "rules_ros2_resolver_deps",
    python_interpreter_target = "@rules_ros2_python_host//:python",
    requirements_lock = "//repositories/private:resolver_requirements_lock.txt",
)

load(
    "@rules_ros2_resolver_deps//:requirements.bzl",
    install_rules_ros2_resolver_deps = "install_deps",
)

install_rules_ros2_resolver_deps()
