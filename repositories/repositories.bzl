"""Handles imports of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
load(
    "@com_github_mvukov_rules_ros2//repositories:ros2_repositories_impl.bzl",
    "ros2_repositories_impl",
)

def ros2_repositories():
    """Imports external/third-party repositories.
    """
    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "84aec9e21cc56fbc7f1335035a71c850d1b9b5cc6ff497306f84cced9a769841",
        strip_prefix = "rules_python-0.23.1",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.23.1.tar.gz",
    )

    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.4.2/bazel-skylib-1.4.2.tar.gz"],
        sha256 = "66ffd9315665bfaafc96b52278f57c7e2dd09f5ede279ea6d39b2be471e7e3aa",
    )

    maybe(
        http_archive,
        name = "fmt",
        build_file = "@com_github_mvukov_rules_ros2//repositories:fmt.BUILD.bazel",
        sha256 = "5cae7072042b3043e12d53d50ef404bbb76949dad1de368d7f993a15c8c05ecc",
        strip_prefix = "fmt-7.1.3",
        url = "https://github.com/fmtlib/fmt/archive/7.1.3.tar.gz",
    )

    maybe(
        http_archive,
        name = "spdlog",
        build_file = "@com_github_mvukov_rules_ros2//repositories:spdlog.BUILD.bazel",
        sha256 = "944d0bd7c763ac721398dca2bb0f3b5ed16f67cef36810ede5061f35a543b4b8",
        strip_prefix = "spdlog-1.8.5",
        url = "https://github.com/gabime/spdlog/archive/v1.8.5.tar.gz",
    )

    maybe(
        http_archive,
        name = "libyaml",
        build_file = "@com_github_mvukov_rules_ros2//repositories:libyaml.BUILD.bazel",
        sha256 = "2c103fc473e904c6fe7580277f1fa16b6d716e54d5e3f32a8913c4850ae03b3f",
        strip_prefix = "libyaml-acd6f6f014c25e46363e718381e0b35205df2d83",
        urls = ["https://github.com/yaml/libyaml/archive/acd6f6f014c25e46363e718381e0b35205df2d83.tar.gz"],
    )

    maybe(
        http_archive,
        name = "pybind11",
        build_file = "@com_github_mvukov_rules_ros2//repositories:pybind11.BUILD.bazel",
        sha256 = "832e2f309c57da9c1e6d4542dedd34b24e4192ecb4d62f6f4866a737454c9970",
        strip_prefix = "pybind11-2.10.4",
        urls = ["https://github.com/pybind/pybind11/archive/refs/tags/v2.10.4.tar.gz"],
    )

    maybe(
        http_archive,
        name = "rules_foreign_cc",
        sha256 = "2a4d07cd64b0719b39a7c12218a3e507672b82a97b98c6a89d38565894cf7c51",
        strip_prefix = "rules_foreign_cc-0.9.0",
        url = "https://github.com/bazelbuild/rules_foreign_cc/archive/refs/tags/0.9.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "com_google_googletest",
        sha256 = "ad7fdba11ea011c1d925b3289cf4af2c66a352e18d4c7264392fead75e919363",
        strip_prefix = "googletest-1.13.0",
        url = "https://github.com/google/googletest/archive/refs/tags/v1.13.0.tar.gz",
    )

    _googletest_deps()

    maybe(
        http_archive,
        name = "tinyxml2",
        build_file = "@com_github_mvukov_rules_ros2//repositories:tinyxml2.BUILD.bazel",
        sha256 = "cc2f1417c308b1f6acc54f88eb70771a0bf65f76282ce5c40e54cfe52952702c",
        strip_prefix = "tinyxml2-9.0.0",
        urls = ["https://github.com/leethomason/tinyxml2/archive/refs/tags/9.0.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "console_bridge",
        build_file = "@com_github_mvukov_rules_ros2//repositories:console_bridge.BUILD.bazel",
        sha256 = "303a619c01a9e14a3c82eb9762b8a428ef5311a6d46353872ab9a904358be4a4",
        strip_prefix = "console_bridge-1.0.2",
        urls = ["https://github.com/ros/console_bridge/archive/1.0.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "readerwriterqueue",
        build_file = "@com_github_mvukov_rules_ros2//repositories:readerwriterqueue.BUILD.bazel",
        sha256 = "fc68f55bbd49a8b646462695e1777fb8f2c0b4f342d5e6574135211312ba56c1",
        strip_prefix = "readerwriterqueue-1.0.6",
        urls = ["https://github.com/cameron314/readerwriterqueue/archive/refs/tags/v1.0.6.tar.gz"],
    )

    maybe(
        http_archive,
        name = "sqlite",
        build_file = "@com_github_mvukov_rules_ros2//repositories:sqlite.BUILD.bazel",
        sha256 = "ad68c1216c3a474cf360c7581a4001e952515b3649342100f2d7ca7c8e313da6",
        strip_prefix = "sqlite-amalgamation-3240000",
        urls = ["https://www.sqlite.org/2018/sqlite-amalgamation-3240000.zip"],
    )

    maybe(
        http_archive,
        name = "zstd",
        build_file = "@com_github_mvukov_rules_ros2//repositories:zstd.BUILD.bazel",
        sha256 = "9c4396cc829cfae319a6e2615202e82aad41372073482fce286fac78646d3ee4",
        strip_prefix = "zstd-1.5.5",
        urls = ["https://github.com/facebook/zstd/releases/download/v1.5.5/zstd-1.5.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "lz4",
        build_file = "@com_github_mvukov_rules_ros2//repositories:lz4.BUILD.bazel",
        sha256 = "4ec935d99aa4950eadfefbd49c9fad863185ac24c32001162c44a683ef61b580",
        strip_prefix = "lz4-1.9.3",
        urls = ["https://github.com/lz4/lz4/archive/refs/tags/v1.9.3.zip"],
    )

    maybe(
        http_archive,
        name = "mcap",
        build_file = "@com_github_mvukov_rules_ros2//repositories:mcap.BUILD.bazel",
        sha256 = "2833f72344308ea58639f3b363a0cf17669580ae7ab435f43f3b104cff6ef548",
        strip_prefix = "mcap-releases-cpp-v0.8.0/cpp/mcap",
        urls = ["https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v0.8.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "yaml_cpp",
        build_file = "@com_github_mvukov_rules_ros2//repositories:yaml_cpp.BUILD.bazel",
        sha256 = "43e6a9fcb146ad871515f0d0873947e5d497a1c9c60c58cb102a97b47208b7c3",
        strip_prefix = "yaml-cpp-yaml-cpp-0.7.0",
        urls = ["https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.7.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "eigen",
        build_file = "@com_github_mvukov_rules_ros2//repositories:eigen.BUILD.bazel",
        sha256 = "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72",
        strip_prefix = "eigen-3.4.0",
        urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros2",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2.BUILD.bazel",
        sha256 = "8223f8de5d4dd3e42251997d0162ad636ed6a1d01eb3e87a079f10f5dda5fdb7",
        strip_prefix = "ros2-release-humble-20230502",
        urls = ["https://github.com/ros2/ros2/archive/refs/tags/release-humble-20230502.tar.gz"],
    )

    ros2_repositories_impl()

    # ros2-devel branch
    maybe(
        http_archive,
        name = "ros2_gps_umd",
        build_file = "@com_github_mvukov_rules_ros2//repositories:gps_umd.BUILD.bazel",
        sha256 = "64a96f93053d0d59e8fcccceab5408a7d666dd813d4c12df139ef24d916f49ab",
        strip_prefix = "gps_umd-fc782811804fafb12ee479a48a2aa2e9ee942e2d",
        urls = ["https://github.com/swri-robotics/gps_umd/archive/fc782811804fafb12ee479a48a2aa2e9ee942e2d.tar.gz"],
    )

    # Needs https://github.com/foxglove/ros-foxglove-bridge/pull/228.
    maybe(
        http_archive,
        name = "foxglove_bridge",
        build_file = "@com_github_mvukov_rules_ros2//repositories:foxglove_bridge.BUILD.bazel",
        sha256 = "264095fff9e51a6a880588e25ecb58fc4d19d0c3da4da21ca3ff2223a0a536ce",
        strip_prefix = "ros-foxglove-bridge-eb0217174750ab6ea7e52aadaf3ff59a022bf153",
        urls = ["https://github.com/foxglove/ros-foxglove-bridge/archive/eb0217174750ab6ea7e52aadaf3ff59a022bf153.tar.gz"],
    )

    maybe(
        http_archive,
        name = "nlohmann_json",
        build_file = "@com_github_mvukov_rules_ros2//repositories:nlohmann_json.BUILD.bazel",
        strip_prefix = "single_include",
        sha256 = "e5c7a9f49a16814be27e4ed0ee900ecd0092bfb7dbfca65b5a421b774dccaaed",
        urls = ["https://github.com/nlohmann/json/releases/download/v3.11.2/include.zip"],
    )

    maybe(
        http_archive,
        name = "websocketpp",
        build_file = "@com_github_mvukov_rules_ros2//repositories:websocketpp.BUILD.bazel",
        sha256 = "6ce889d85ecdc2d8fa07408d6787e7352510750daa66b5ad44aacb47bea76755",
        strip_prefix = "websocketpp-0.8.2",
        urls = ["https://github.com/zaphoyd/websocketpp/archive/refs/tags/0.8.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "asio",
        build_file = "@com_github_mvukov_rules_ros2//repositories:asio.BUILD.bazel",
        sha256 = "b31c63867daaba0e460ee2c85dc508a52c81db0a7318e0d2147f444b26f80ed7",
        strip_prefix = "asio-asio-1-27-0/asio",
        urls = ["https://github.com/chriskohlhoff/asio/archive/refs/tags/asio-1-27-0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "openssl",
        build_file = "@com_github_mvukov_rules_ros2//repositories:openssl.BUILD.bazel",
        sha256 = "f89199be8b23ca45fc7cb9f1d8d3ee67312318286ad030f5316aca6462db6c96",
        strip_prefix = "openssl-1.1.1m",
        urls = ["https://www.openssl.org/source/openssl-1.1.1m.tar.gz"],
    )

    maybe(
        http_archive,
        name = "zlib",
        build_file = "@com_github_mvukov_rules_ros2//repositories:zlib.BUILD.bazel",
        sha256 = "d14c38e313afc35a9a8760dadf26042f51ea0f5d154b0630a31da0540107fb98",
        strip_prefix = "zlib-1.2.13",
        urls = [
            "https://github.com/madler/zlib/releases/download/v1.2.13/zlib-1.2.13.tar.xz",
            "https://zlib.net/zlib-1.2.13.tar.xz",
        ],
    )

    maybe(
        http_archive,
        name = "ros2_xacro",
        build_file = "@com_github_mvukov_rules_ros2//repositories:xacro.BUILD.bazel",
        sha256 = "a8802a5b48f7479bae1238e822ac4ebb47660221eb9bc40a608e899d60f3f7e4",
        strip_prefix = "xacro-2.0.9",
        urls = ["https://github.com/ros/xacro/archive/refs/tags/2.0.9.tar.gz"],
    )

    # Version copied from https://github.com/ros2/orocos_kdl_vendor/blob/0.2.5/orocos_kdl_vendor/CMakeLists.txt#L58.
    maybe(
        http_archive,
        name = "orocos_kdl",
        build_file = "@com_github_mvukov_rules_ros2//repositories:orocos_kdl.BUILD.bazel",
        sha256 = "22df47f63d91d014af2675029c23da83748575c12a6481fda3ed9235907cc259",
        strip_prefix = "orocos_kinematics_dynamics-507de66205e14b12c8c65f25eafc05c4dc66e21e",
        urls = ["https://github.com/orocos/orocos_kinematics_dynamics/archive/507de66205e14b12c8c65f25eafc05c4dc66e21e.tar.gz"],
    )

    maybe(
        http_archive,
        name = "tinyxml",
        build_file = "@com_github_mvukov_rules_ros2//repositories:tinyxml.BUILD.bazel",
        sha256 = "15bdfdcec58a7da30adc87ac2b078e4417dbe5392f3afb719f9ba6d062645593",
        urls = [
            "http://archive.ubuntu.com/ubuntu/pool/universe/t/tinyxml/tinyxml_2.6.2.orig.tar.gz",
        ],
    )

    maybe(
        http_archive,
        name = "ros2_urdf",
        build_file = "@com_github_mvukov_rules_ros2//repositories:urdf.BUILD.bazel",
        sha256 = "a762eb57dc7f60b9ada0240fd7c609f0dc5028ef0b4b65972daf91e009e52cf6",
        strip_prefix = "urdf-2.6.0",
        urls = ["https://github.com/ros2/urdf/archive/refs/tags/2.6.0.tar.gz"],
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:urdf_plugin_loader_fix.patch"],
    )

    maybe(
        http_archive,
        name = "ros2_diagnostics",
        build_file = "@com_github_mvukov_rules_ros2//repositories:diagnostics.BUILD.bazel",
        sha256 = "a723dae7acf0f00ee643c076c7c81299be0254919f29225ec7a89dc14cb8ea6f",
        strip_prefix = "diagnostics-9f402787ea2c9b3dd4d7e51a9986810e8a3400ba",
        urls = ["https://github.com/ros/diagnostics/archive/9f402787ea2c9b3dd4d7e51a9986810e8a3400ba.zip"],
    )

def _googletest_deps():
    """Lists implicit googletest deps.

    Necessary such that e.g. `bazel fetch //...` can work.
    The versions below taken from https://github.com/google/googletest/blob/v1.13.0/WORKSPACE.

    TODO(mvukov) More recent commits in googletest have googletest_deps.bzl.
        Integrate once a new release is available.
    """
    maybe(
        http_archive,
        name = "com_google_absl",  # 2023-01-10T21:08:25Z
        sha256 = "f9a4e749f42c386a32a90fddf0e2913ed408d10c42f7f33ccf4c59ac4f0d1d05",
        strip_prefix = "abseil-cpp-52835439ca90d86b27bf8cd1708296e95604d724",
        urls = ["https://github.com/abseil/abseil-cpp/archive/52835439ca90d86b27bf8cd1708296e95604d724.zip"],
    )

    # Note this must use a commit from the `abseil` branch of the RE2 project.
    # https://github.com/google/re2/tree/abseil
    maybe(
        http_archive,
        name = "com_googlesource_code_re2",  # 2022-12-21T14:29:10Z
        sha256 = "b9ce3a51beebb38534d11d40f8928d40509b9e18a735f6a4a97ad3d014c87cb5",
        strip_prefix = "re2-d0b1f8f2ecc2ea74956c7608b6f915175314ff0e",
        urls = ["https://github.com/google/re2/archive/d0b1f8f2ecc2ea74956c7608b6f915175314ff0e.zip"],
    )
