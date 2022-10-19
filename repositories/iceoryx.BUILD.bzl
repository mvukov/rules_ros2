load("@rules_foreign_cc//foreign_cc:cmake.bzl", "cmake")

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
        "BUILD_SHARED_LIBS": "OFF",
        "CCACHE": "OFF",
        "CMAKE_CXX_FLAGS": "-I$EXT_BUILD_DEPS/acl/include -L$EXT_BUILD_DEPS/acl/lib",
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
    deps = ["@acl//:acl"],
)

cc_binary(
    name = "shared_memory_manager",
    srcs = ["iceoryx_posh/source/roudi/application/roudi_main.cpp"],
    deps = ["@iceoryx//:iceoryx"],
    linkopts = [
        "-lpthread",
        "-lrt",
        "-latomic",
    ],
    linkstatic = True,
    visibility = ["//visibility:public"],
    copts = ["-std=c++14"],
)
