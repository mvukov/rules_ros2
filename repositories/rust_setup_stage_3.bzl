load("@rules_rust//crate_universe:defs.bzl", "crate", "crates_repository")

def rust_setup_stage_3(**kwargs):
    crates_repository(
        name = "rules_ros2_crate_index",
        cargo_lockfile = "@com_github_mvukov_rules_ros2//repositories/rust:Cargo.lock",
        # Keep specs in sync with MODULE.bazel
        packages = {
            "futures": crate.spec(version = "0.3"),
            "serde": crate.spec(features = ["derive"], version = "1"),
            "serde-big-array": crate.spec(version = "0.5"),
            # Not used by rclrs, used for testing.
            "serde_json": crate.spec(version = "1"),
            # Not used by rclrs, but handy to have.
            "signal-hook": crate.spec(version = "0.3"),
        },
        **kwargs
    )
