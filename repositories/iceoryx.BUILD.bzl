load("@rules_foreign_cc//foreign_cc:cmake.bzl", "cmake")

filegroup(
    name = "iceoryx_hoofs_srcs",
    srcs = glob(["iceoryx_hoofs/**"]),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "all_srcs",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)

cmake(
    name = "iceoryx",
    build_args = [
        "--",  # <- pass options to the native tool
        "-j4",
    ],
    cache_entries = {
        "CMAKE_BUILD_TYPE": "Release",
    },
    lib_source = ":all_srcs",
    out_static_libs = [
        "libiceoryx_binding_c.a",
        "libiceoryx_posh.a",
        "libiceoryx_posh_roudi.a",
        "libiceoryx_posh_gateway.a",
        "libiceoryx_posh_config.a",
        "libiceoryx_hoofs.a",
        "libiceoryx_platform.a",
    ],
    out_binaries = ["iox-roudi"],
    out_include_dir = "include/iceoryx/v2.0.2/",
    visibility = ["//visibility:public"],
    working_directory = "iceoryx_meta",
    tags = ["requires-network"],
)
