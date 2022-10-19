load("@rules_foreign_cc//foreign_cc:defs.bzl", "configure_make")

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)

configure_make(
    name = "acl",
    args = [
        "-j4",
    ],
    configure_in_place = True,
    lib_source = "@acl//:all_srcs",
    out_static_libs = [
        "libacl.a",
    ],
    out_include_dir = "include",
    visibility = ["//visibility:public"],
    deps = ["@attr//:attr"],
)
