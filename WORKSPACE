workspace(name = "com_github_mvukov_rules_ros2")

load("//repositories:repositories.bzl", "ros2_repositories", "ros2_workspace_repositories")

ros2_workspace_repositories()

ros2_repositories()

load("//repositories:deps.bzl", "ros2_deps")

ros2_deps()

load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")

py_repositories()

python_register_toolchains(
    name = "rules_ros2_python_3_12",
    python_version = "3.12",
)

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_pycross",
    sha256 = "4acc6eb0f04baf94e6a864e6a199a050e9465967b8b8e900523cc2f4214e7937",
    strip_prefix = "rules_pycross-0.8.1",
    url = "https://github.com/jvolkman/rules_pycross/releases/download/v0.8.1/rules_pycross-v0.8.1.tar.gz",
)

load("@rules_pycross//pycross:repositories.bzl", "rules_pycross_dependencies")

rules_pycross_dependencies()

load("@rules_pycross//pycross:workspace.bzl", "lock_repo_model_poetry", "pycross_lock_repo")

pycross_lock_repo(
    name = "rules_ros2_py_deps",
    lock_model = lock_repo_model_poetry(
        lock_file = "@com_github_mvukov_rules_ros2//:poetry.lock",
        project_file = "@com_github_mvukov_rules_ros2//:pyproject.toml",
    ),
)

load(
    "@rules_ros2_py_deps//:requirements.bzl",
    install_rules_ros2_py_deps = "install_deps",
)

install_rules_ros2_py_deps()

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
    python_interpreter_target = "@rules_ros2_python_3_12_host//:python",
    requirements_lock = "//repositories/private:resolver_requirements_lock.txt",
)

load(
    "@rules_ros2_resolver_deps//:requirements.bzl",
    install_rules_ros2_resolver_deps = "install_deps",
)

install_rules_ros2_resolver_deps()
