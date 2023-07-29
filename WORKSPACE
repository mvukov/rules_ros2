workspace(name = "com_github_mvukov_rules_ros2")

load("//repositories:repositories.bzl", "ros2_repositories")

ros2_repositories()

load("//repositories:deps.bzl", "PIP_ANNOTATIONS", "ros2_deps")

ros2_deps()

load("@rules_python//python:repositories.bzl", "python_register_toolchains")

python_register_toolchains(
    name = "rules_ros2_python",
    python_version = "3.8.13",
)

load("@rules_python//python:pip.bzl", "pip_parse")
load("@rules_ros2_python//:defs.bzl", python_interpreter_target = "interpreter")

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

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "aspect_gcc_toolchain",
    sha256 = "8850373f24d3f8bb6e8f36e3e8e7edc93d948964f8f201e920af2c8ffba2002c",
    strip_prefix = "gcc-toolchain-4bd1f94536ee92b7c49673931773038d923ee86e",
    url = "https://github.com/aspect-build/gcc-toolchain/archive/4bd1f94536ee92b7c49673931773038d923ee86e.tar.gz",
)

load("@aspect_gcc_toolchain//toolchain:repositories.bzl", "gcc_toolchain_dependencies")

gcc_toolchain_dependencies()

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

load("@aspect_bazel_lib//lib:repositories.bzl", "aspect_bazel_lib_dependencies")

aspect_bazel_lib_dependencies()

load("@aspect_gcc_toolchain//toolchain:defs.bzl", "ARCHS", "gcc_register_toolchain")

gcc_register_toolchain(
    name = "gcc_toolchain_aarch64",
    target_arch = ARCHS.aarch64,
)