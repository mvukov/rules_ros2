load("@rules_rust//crate_universe:defs.bzl", "crate", "crates_repository")

def ground_control_setup_stage_1(**kwargs):
    crates_repository(
        name = "ground_control_crate_index",
        cargo_lockfile = "@com_github_mvukov_rules_ros2//repositories:ground_control_cargo.lock",
        packages = {
            "clap": crate.spec(features = ["derive"], version = "4.1.8"),
            "color-eyre": crate.spec(default_features = False, version = "0.6.2"),
            "command-group": crate.spec(features = ["with-tokio"], version = "2.0.0"),
            "console": crate.spec(default_features = False, features = ["ansi-parsing"], version = "0.15.2"),
            "nix": crate.spec(default_features = False, features = ["signal"], version = "0.26.1"),
            "once_cell": crate.spec(version = "1.16.0"),
            "regex": crate.spec(version = "1.6.0"),
            "serde": crate.spec(features = ["derive"], version = "1.0.126"),
            "thiserror": crate.spec(version = "1.0"),
            "time": crate.spec(features = ["formatting", "macros"], version = "0.3.17"),
            "tokio": crate.spec(features = ["fs", "macros", "process", "rt-multi-thread", "signal", "sync"], version = "1.26.0"),
            "toml": crate.spec(version = "0.5"),
            "tracing": crate.spec(version = "0.1"),
            "tracing-subscriber": crate.spec(default_features = False, features = ["env-filter", "fmt", "std"], version = "0.3"),
            "users": crate.spec(version = "0.11.0"),
        },
        **kwargs
    )
