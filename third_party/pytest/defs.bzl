load("@rules_python//python:defs.bzl", "py_test")
load("@rules_ros2_pip_deps//:requirements.bzl", "requirement")

def py_pytest_test(name, srcs, deps = [], args = [], **kwargs):
    py_test(
        name = name,
        srcs = [
            "//third_party/pytest:pytest_wrapper.py",
        ] + srcs,
        main = "//third_party/pytest:pytest_wrapper.py",
        args = [
            "-ra",
            "-vv",
        ] + args + ["$(location :%s)" % x for x in srcs],
        deps = deps + [
            "//third_party/pytest:pytest_wrapper",
            requirement("coverage"),
            requirement("pytest"),
            requirement("pytest-cov"),
        ],
        **kwargs
    )
