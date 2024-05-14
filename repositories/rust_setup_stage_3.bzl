load("@rules_rust//crate_universe:defs.bzl", "crate", "crates_repository", "render_config")

def rust_setup_stage_3():
    crates_repository(
        name = "rules_ros2_crate_index",
        cargo_lockfile = "@com_github_mvukov_rules_ros2//repositories/rust:Cargo.lock",
        lockfile = "@com_github_mvukov_rules_ros2//repositories/rust:Cargo.Bazel.lock",
        packages = {
            "futures": crate.spec(
                version = "0.3",
            ),
            "serde": crate.spec(
                features = ["derive"],
                version = "1",
            ),
        },
        # Setting the default package name to `""` forces the use of the macros defined in this repository
        # to always use the root package when looking for dependencies or aliases. This should be considered
        # optional as the repository also exposes alises for easy access to all dependencies.
        render_config = render_config(
            default_package_name = "",
        ),
    )
