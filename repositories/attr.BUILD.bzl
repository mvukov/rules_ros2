load("@rules_foreign_cc//foreign_cc:defs.bzl", "configure_make")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)

configure_make(
    name = "attr",
    args = [
        "-j4",
    ],
    configure_in_place = True,
    lib_source = ":all_srcs",
    out_static_libs = ["libattr.a"],
    out_include_dir = "include",
    visibility = ["//visibility:public"],
    targets = ["", "install", "install-dev", "install-lib"],
)