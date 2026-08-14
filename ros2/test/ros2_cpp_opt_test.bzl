load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_test")
load("@with_cfg.bzl", "with_cfg")

ros2_cpp_opt_test, _ros2_cpp_opt_test_internal = with_cfg(ros2_cpp_test).set("compilation_mode", "opt").build()
