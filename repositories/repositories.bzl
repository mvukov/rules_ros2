"""Handles imports of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
load(
    "@com_github_mvukov_rules_ros2//repositories:ros2_repositories_impl.bzl",
    "ros2_repositories_impl",
)

def ros2_workspace_repositories():
    """Imports dependent third-party repositories for the non-blzmod (hence, workspace-) version of the repository.

    In particular, imports third-party package repositories excluding ROS 2 packages. ROS 2-specific repositories are imported with `ros2_repositories()` macro.
    """
    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "9c6e26911a79fbf510a8f06d8eedb40f412023cf7fa6d1461def27116bff022c",
        strip_prefix = "rules_python-1.1.0",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/1.1.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.7.1/bazel-skylib-1.7.1.tar.gz"],
        sha256 = "bc283cdfcd526a52c3201279cda4bc298652efa898b10b4db0837dc51652756f",
    )

    maybe(
        http_archive,
        name = "fmt",
        build_file = "@com_github_mvukov_rules_ros2//repositories:fmt.BUILD.bazel",
        sha256 = "1250e4cc58bf06ee631567523f48848dc4596133e163f02615c97f78bab6c811",
        strip_prefix = "fmt-10.2.1",
        url = "https://github.com/fmtlib/fmt/archive/10.2.1.tar.gz",
    )

    maybe(
        http_archive,
        name = "spdlog",
        build_file = "@com_github_mvukov_rules_ros2//repositories:spdlog.BUILD.bazel",
        sha256 = "9962648c9b4f1a7bbc76fd8d9172555bad1871fdb14ff4f842ef87949682caa5",
        strip_prefix = "spdlog-1.15.0",
        url = "https://github.com/gabime/spdlog/archive/v1.15.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "libyaml",
        build_file = "@com_github_mvukov_rules_ros2//repositories:libyaml.BUILD.bazel",
        sha256 = "c642ae9b75fee120b2d96c712538bd2cf283228d2337df2cf2988e3c02678ef4",
        strip_prefix = "yaml-0.2.5",
        urls = ["https://github.com/yaml/libyaml/releases/download/0.2.5/yaml-0.2.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "pybind11",
        build_file = "@com_github_mvukov_rules_ros2//repositories:pybind11.BUILD.bazel",
        sha256 = "e08cb87f4773da97fa7b5f035de8763abc656d87d5773e62f6da0587d1f0ec20",
        strip_prefix = "pybind11-2.13.6",
        urls = ["https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz"],
    )

    maybe(
        http_archive,
        name = "bazel_features",
        sha256 = "b4b145c19e08fd48337f53c383db46398d0a810002907ff0c590762d926e05be",
        strip_prefix = "bazel_features-1.18.0",
        url = "https://github.com/bazel-contrib/bazel_features/releases/download/v1.18.0/bazel_features-v1.18.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "rules_foreign_cc",
        sha256 = "8e5605dc2d16a4229cb8fbe398514b10528553ed4f5f7737b663fdd92f48e1c2",
        strip_prefix = "rules_foreign_cc-0.13.0",
        url = "https://github.com/bazelbuild/rules_foreign_cc/archive/refs/tags/0.13.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "googletest",
        sha256 = "7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926",
        strip_prefix = "googletest-1.15.2",
        url = "https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz",
    )

    maybe(
        http_archive,
        name = "tinyxml2",
        build_file = "@com_github_mvukov_rules_ros2//repositories:tinyxml2.BUILD.bazel",
        sha256 = "3bdf15128ba16686e69bce256cc468e76c7b94ff2c7f391cc5ec09e40bff3839",
        strip_prefix = "tinyxml2-10.0.0",
        urls = ["https://github.com/leethomason/tinyxml2/archive/refs/tags/10.0.0.tar.gz"],
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
        name = "sqlite3",
        build_file = "@com_github_mvukov_rules_ros2//repositories:sqlite3.BUILD.bazel",
        sha256 = "ad68c1216c3a474cf360c7581a4001e952515b3649342100f2d7ca7c8e313da6",
        strip_prefix = "sqlite-amalgamation-3240000",
        urls = ["https://www.sqlite.org/2018/sqlite-amalgamation-3240000.zip"],
    )

    maybe(
        http_archive,
        name = "nlohmann_json",
        sha256 = "a22461d13119ac5c78f205d3df1db13403e58ce1bb1794edc9313677313f4a9d",
        urls = ["https://github.com/nlohmann/json/releases/download/v3.11.3/include.zip"],
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
        sha256 = "530540f973498c2d297771af1bc852f69b27509bbb56bc7ac3309c928373286f",
        strip_prefix = "asio-asio-1-31-0",
        url = "https://github.com/chriskohlhoff/asio/archive/refs/tags/asio-1-31-0.tar.gz",
    )

    # We're pointing at hedronvision's mirror of google/boringssl:main-with-bazel to get
    # Renovate auto-update. Otherwise, Renovate will keep moving us back to main, which doesn't
    # support Bazel.
    maybe(
        http_archive,
        name = "boringssl",
        sha256 = "d38af313617ce2e952a7af6ba80e2cd87520b5c1c355316ea4222a2a3edbcd21",
        strip_prefix = "boringssl-266308793d4d0d1f20c817efda8da00bf393bfd6",
        urls = ["https://github.com/hedronvision/boringssl/archive/266308793d4d0d1f20c817efda8da00bf393bfd6.tar.gz"],
    )

    maybe(
        http_archive,
        name = "zlib",
        build_file = "@com_github_mvukov_rules_ros2//repositories:zlib.BUILD.bazel",
        sha256 = "8a9ba2898e1d0d774eca6ba5b4627a11e5588ba85c8851336eb38de4683050a7",
        strip_prefix = "zlib-1.3",
        urls = ["https://github.com/madler/zlib/releases/download/v1.3/zlib-1.3.tar.xz"],
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
        name = "curl",
        build_file = "@com_github_mvukov_rules_ros2//repositories:curl.BUILD.bazel",
        sha256 = "77c0e1cd35ab5b45b659645a93b46d660224d0024f1185e8a95cdb27ae3d787d",
        strip_prefix = "curl-8.8.0",
        urls = ["https://github.com/curl/curl/releases/download/curl-8_8_0/curl-8.8.0.tar.gz"],
        patch_args = ["-p1"],
        patches = ["@com_github_mvukov_rules_ros2//repositories/patches:curl_fix_openssl.patch"],
    )

    maybe(
        http_archive,
        name = "zstd",
        build_file = "@com_github_mvukov_rules_ros2//repositories:zstd.BUILD.bazel",
        sha256 = "8c29e06cf42aacc1eafc4077ae2ec6c6fcb96a626157e0593d5e82a34fd403c1",
        strip_prefix = "zstd-1.5.6",
        urls = ["https://github.com/facebook/zstd/releases/download/v1.5.6/zstd-1.5.6.tar.gz"],
    )

    maybe(
        http_archive,
        name = "lz4",
        build_file = "@com_github_mvukov_rules_ros2//repositories:lz4.BUILD.bazel",
        sha256 = "0b0e3aa07c8c063ddf40b082bdf7e37a1562bda40a0ff5272957f3e987e0e54b",
        strip_prefix = "lz4-1.9.4",
        urls = ["https://github.com/lz4/lz4/archive/refs/tags/v1.9.4.tar.gz"],
    )

    maybe(
        http_archive,
        name = "yaml-cpp",
        build_file = "@com_github_mvukov_rules_ros2//repositories:yaml-cpp.BUILD.bazel",
        sha256 = "fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16",
        strip_prefix = "yaml-cpp-0.8.0",
        urls = ["https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz"],
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
        name = "rules_license",
        sha256 = "26d4021f6898e23b82ef953078389dd49ac2b5618ac564ade4ef87cced147b38",
        urls = [
            "https://github.com/bazelbuild/rules_license/releases/download/1.0.0/rules_license-1.0.0.tar.gz",
        ],
    )

def ros2_repositories():
    """Import ROS 2 repositories."""

    maybe(
        http_archive,
        name = "ros2",
        build_file = "@com_github_mvukov_rules_ros2//repositories:ros2.BUILD.bazel",
        sha256 = "67757489197ea587d1832cccc2318f38468a760dfcf7627cffd78e2a4e25ac4a",
        strip_prefix = "ros2-release-humble-20241205",
        urls = ["https://github.com/ros2/ros2/archive/refs/tags/release-humble-20241205.tar.gz"],
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

    maybe(
        http_archive,
        name = "foxglove_bridge",
        build_file = "@com_github_mvukov_rules_ros2//repositories:foxglove_bridge.BUILD.bazel",
        sha256 = "9548f6a53794cfcd6dcae570e6e82aa2c7670d177269ecf612838655d7ba7dcc",
        strip_prefix = "ros-foxglove-bridge-0.7.9",
        urls = ["https://github.com/foxglove/ros-foxglove-bridge/archive/refs/tags/0.7.9.tar.gz"],
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
        name = "mcap",
        build_file = "@com_github_mvukov_rules_ros2//repositories:mcap.BUILD.bazel",
        sha256 = "69bcd33e1590201ea180e9e6ba8a22f3e8e7e2283e9ebcbff6a2c7134d8341db",
        strip_prefix = "mcap-releases-cpp-v1.4.1/cpp/mcap",
        urls = ["https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v1.4.1.tar.gz"],
    )

    # NOTE: Use the humble branch.
    maybe(
        http_archive,
        name = "ros2_rcl_logging_syslog",
        build_file = "@com_github_mvukov_rules_ros2//repositories:rcl_logging_syslog.BUILD.bazel",
        sha256 = "89039a8d05d1d14ccb85a3d065871d54cce831522bd8aa687e27eb6afd333d07",
        strip_prefix = "rcl_logging_syslog-e63257f2d5ca693f286bbcedf2b23720675b7f73",
        urls = ["https://github.com/fujitatomoya/rcl_logging_syslog/archive/e63257f2d5ca693f286bbcedf2b23720675b7f73.zip"],
    )
