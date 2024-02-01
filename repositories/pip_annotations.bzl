load("@rules_python//python:pip.bzl", "package_annotation")

# Keep this in sync with MODULE.bazel
PIP_ANNOTATIONS = {
    "numpy": package_annotation(
        additive_build_content = """\
cc_library(
    name = "headers",
    hdrs = glob(["site-packages/numpy/core/include/numpy/**/*.h"]),
    includes = ["site-packages/numpy/core/include"],
    deps = ["@rules_python//python/cc:current_py_cc_headers"],
)
""",
    ),
}
