load("@rules_rust//crate_universe:defs.bzl", "crate", "crates_repository")

def rust_setup_stage_3(**kwargs):
    crates_repository(
        name = "rules_ros2_crate_index",
        cargo_lockfile = "@com_github_mvukov_rules_ros2//repositories/rust:Cargo.lock",
        # Keep specs in sync with MODULE.bazel
        packages = {
            "async-std": crate.spec(version = "1.13"),
            "async-stream": crate.spec(version = "0.3.6"),
            "futures": crate.spec(version = "0.3"),
            "futures-lite": crate.spec(features = ["race", "std"], version = "2.6"),
            "serde": crate.spec(features = ["derive"], version = "1"),
            "serde-big-array": crate.spec(version = "0.5"),
            # Not used by rclrs, used for testing.
            "serde_json": crate.spec(version = "1"),
            # Not used by rclrs, but handy to have.
            "signal-hook": crate.spec(version = "0.3"),
            "tokio": crate.spec(features = ["sync"], version = "1"),
            "tokio-stream": crate.spec(version = "0.1"),
            "uuid": crate.spec(features = ["v4"], version = "1"),
        },
        **kwargs
    )
