# Adapted from https://gist.github.com/betaboon/c1dd785b5ba468b4df4e382eafff969a

load("@rules_python//python:defs.bzl", "py_test")
load("@rules_ros2_pip_deps//:requirements.bzl", "requirement")

def py_pytest_test(name, srcs, deps = None, args = None, **kwargs):
    deps = deps or []
    args = args or []
    py_test(
        name = name,
        srcs = [
            "@com_github_mvukov_rules_ros2//third_party/pytest:pytest_wrapper.py",
        ] + srcs,
        main = "@com_github_mvukov_rules_ros2//third_party/pytest:pytest_wrapper.py",
        args = [
            "-ra",
            "-vv",
        ] + args + ["$(location :%s)" % x for x in srcs],
        deps = deps + [
            "@com_github_mvukov_rules_ros2//third_party/pytest:pytest_wrapper",
            requirement("coverage"),
            requirement("pytest"),
            requirement("pytest-cov"),
        ],
        **kwargs
    )
