load("@rules_rust//crate_universe:defs.bzl", "crate", "crates_repository")

# clap = { version = "4.1.8", features = ["derive"] }
# color-eyre = { version = "0.6.2", default-features = false }
# command-group = { version = "2.0.0", features = ["with-tokio"] }
# console = { version = "0.15.2", default-features = false, features = ["ansi-parsing"] }
# nix = { version = "0.26.1", default-features = false, features = ["signal"] }
# once_cell = "1.16.0"
# regex = "1.6.0"
# serde = { version = "1.0.126", features = ["derive"] }
# thiserror = "1.0"
# time = { version = "0.3.17", features = ["formatting", "macros"] }
# tokio = { version = "1.26.0", features = ["fs", "macros", "process", "rt-multi-thread", "signal", "sync"] }
# toml = "0.5"
# tracing = "0.1"
# tracing-subscriber = { version = "0.3", default-features = false, features = ["env-filter", "fmt", "std"] }
users = "0.11.0"

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
