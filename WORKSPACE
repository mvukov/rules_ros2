workspace(name = "com_github_mvukov_rules_ros2")

load("//repositories:repositories.bzl", "ros2_repositories")

ros2_repositories()

load("//repositories:deps.bzl", "ros2_deps")

PYTHON_INTERPRETER = "python3.8"

ros2_deps(
    python_interpreter = PYTHON_INTERPRETER,
    python_requirements_lock = "//:requirements_lock.txt",
)

load(
    "@rules_ros2_pip_deps//:requirements.bzl",
    install_rules_ros2_pip_deps = "install_deps",
)

install_rules_ros2_pip_deps()
