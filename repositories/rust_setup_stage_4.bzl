load("@rules_ros2_crate_index//:defs.bzl", "crate_repositories")

def rust_setup_stage_4():
    crate_repositories()
