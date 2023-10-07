load("@rules_python//python:pip.bzl", "package_annotation")

PIP_ANNOTATIONS = {
    "numpy": package_annotation(
        additive_build_content = """\
cc_library(
    name = "headers",
    hdrs = glob(["site-packages/numpy/core/include/numpy/**/*.h"]),
    includes = ["site-packages/numpy/core/include"],
    deps = ["@rules_ros2_python//:python_headers"],
)
""",
    ),
}
