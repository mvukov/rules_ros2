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

# Below are internal deps and deps needed for examples.

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "io_bazel_rules_docker",
    sha256 = "b1e80761a8a8243d03ebca8845e9cc1ba6c82ce7c5179ce2b295cd36f7e394bf",
    urls = ["https://github.com/bazelbuild/rules_docker/releases/download/v0.25.0/rules_docker-v0.25.0.tar.gz"],
)

load(
    "@io_bazel_rules_docker//repositories:repositories.bzl",
    container_repositories = "repositories",
)

container_repositories()

load("@io_bazel_rules_docker//repositories:deps.bzl", container_deps = "deps")

container_deps()

load(
    "@io_bazel_rules_docker//container:container.bzl",
    "container_pull",
)

container_pull(
    name = "ros_deploy_base",
    digest = "sha256:54967c8f59e8607cd4a40c0d614b3391bf71112482f2e344d93ff455f60b3723",
    registry = "docker.io",
    repository = "mvukov/ros-deploy-base",
)

load("@rules_python//python:pip.bzl", "pip_parse")

pip_parse(
    name = "rules_ros2_resolver_deps",
    python_interpreter = PYTHON_INTERPRETER,
    requirements_lock = "//repositories:resolver_requirements_lock.txt",
)

load(
    "@rules_ros2_resolver_deps//:requirements.bzl",
    install_rules_ros2_resolver_deps = "install_deps",
)

install_rules_ros2_resolver_deps()
