load("@rules_rust//crate_universe:repositories.bzl", "crate_universe_dependencies")
load("@rules_rust//rust:repositories.bzl", "rules_rust_dependencies", "rust_register_toolchains")
load("@rules_rust_bindgen//:repositories.bzl", "rust_bindgen_dependencies")
load("//repositories:clang_configure.bzl", "clang_configure")

def rust_setup_stage_2():
    rules_rust_dependencies()
    rust_register_toolchains(edition = "2021")

    clang_configure(name = "rules_ros2_config_clang")

    rust_bindgen_dependencies()
    native.register_toolchains("@com_github_mvukov_rules_ros2//repositories/rust:bindgen_toolchain")

    crate_universe_dependencies()
