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
load("@rules_ros2_python//:defs.bzl", python_interpreter_target = "interpreter")
load("//repositories:pip_annotations.bzl", "PIP_ANNOTATIONS")

pip_parse(
    name = "rules_ros2_pip_deps",
    annotations = PIP_ANNOTATIONS,
    python_interpreter_target = python_interpreter_target,
    requirements_lock = "@com_github_mvukov_rules_ros2//:requirements_lock.txt",
)

load(
    "@rules_ros2_pip_deps//:requirements.bzl",
    install_rules_ros2_pip_deps = "install_deps",
)

install_rules_ros2_pip_deps()

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_rust",
    integrity = "sha256-JLN47ZcAbx9wEr5Jiib4HduZATGLiDgK7oUi/fvotzU=",
    urls = ["https://github.com/bazelbuild/rules_rust/releases/download/0.42.1/rules_rust-v0.42.1.tar.gz"],
)

load("@rules_rust//rust:repositories.bzl", "rules_rust_dependencies", "rust_register_toolchains")

rules_rust_dependencies()

rust_register_toolchains()

load("@rules_rust//crate_universe:repositories.bzl", "crate_universe_dependencies")

crate_universe_dependencies()

load("@rules_rust//crate_universe:defs.bzl", "crate", "crates_repository", "render_config")

crates_repository(
    name = "rules_ros2_crate_index",
    cargo_lockfile = "//:Cargo.lock",
    lockfile = "//:Cargo.Bazel.lock",
    packages = {
        "bindgen": crate.spec(
            version = "0.66.1",
        ),
        "cfg-if": crate.spec(
            version = "1.0.0",
        ),
        "futures": crate.spec(
            version = "0.3",
        ),
    },
    # Setting the default package name to `""` forces the use of the macros defined in this repository
    # to always use the root package when looking for dependencies or aliases. This should be considered
    # optional as the repository also exposes alises for easy access to all dependencies.
    render_config = render_config(
        default_package_name = "",
    ),
)

load("@rules_ros2_crate_index//:defs.bzl", rules_ros2_crate_repositories = "crate_repositories")

rules_ros2_crate_repositories()

load("//repositories:clang_configure.bzl", "clang_configure")

clang_configure(
    name = "rules_ros2_config_clang",
)

load("@rules_rust//bindgen:repositories.bzl", "rust_bindgen_dependencies", "rust_bindgen_register_toolchains")

rust_bindgen_dependencies()

register_toolchains("//:bindgen_toolchain")

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
