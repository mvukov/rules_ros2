load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

# Force re-evaluation
def third_party_repositories():
    maybe(
        http_archive,
        name = "fmt",
        build_file = "@com_github_mvukov_rules_ros2//repositories:fmt.BUILD.bazel",
        sha256 = "ea7de4299689e12b6dddd392f9896f08fb0777ac7168897a244a6d6085043fea",
        strip_prefix = "fmt-12.1.0",
        url = "https://github.com/fmtlib/fmt/archive/12.1.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "spdlog",
        build_file = "@com_github_mvukov_rules_ros2//repositories:spdlog.BUILD.bazel",
        sha256 = "15a04e69c222eb6c01094b5c7ff8a249b36bb22788d72519646fb85feb267e67",
        strip_prefix = "spdlog-1.15.3",
        url = "https://github.com/gabime/spdlog/archive/v1.15.3.tar.gz",
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
        sha256 = "741633da746b7c738bb71f1854f957b9da660bcd2dce68d71949037f0969d0ca",
        strip_prefix = "pybind11-3.0.1",
        urls = ["https://github.com/pybind/pybind11/archive/refs/tags/v3.0.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "tinyxml2",
        build_file = "@com_github_mvukov_rules_ros2//repositories:tinyxml2.BUILD.bazel",
        sha256 = "5556deb5081fb246ee92afae73efd943c889cef0cafea92b0b82422d6a18f289",
        strip_prefix = "tinyxml2-11.0.0",
        urls = ["https://github.com/leethomason/tinyxml2/archive/refs/tags/11.0.0.tar.gz"],
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

    maybe(
        http_archive,
        name = "boringssl",
        sha256 = "fb1326e78f644859de1a92bbea6671a88e91a7658f5bfc5080704349bb4c2eff",
        strip_prefix = "boringssl-3a2c54253492bbc471c67b6e24a39845bc4afb79",
        urls = ["https://github.com/hedronvision/boringssl/archive/3a2c54253492bbc471c67b6e24a39845bc4afb79.tar.gz"],
    )

    maybe(
        http_archive,
        name = "zlib",
        build_file = "@com_github_mvukov_rules_ros2//repositories:zlib.BUILD.bazel",
        sha256 = "9a93b2b7dfdac77ceba5a558a580e74667dd6fede4585b91eefb60f03b72df23",
        strip_prefix = "zlib-1.3.1",
        urls = ["https://github.com/madler/zlib/releases/download/v1.3.1/zlib-1.3.1.tar.gz"],
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
        sha256 = "eb33e51f49a15e023950cd7825ca74a4a2b43db8354825ac24fc1b7ee09e6fa3",
        strip_prefix = "zstd-1.5.7",
        urls = ["https://github.com/facebook/zstd/releases/download/v1.5.7/zstd-1.5.7.tar.gz"],
    )

    maybe(
        http_archive,
        name = "lz4",
        build_file = "@com_github_mvukov_rules_ros2//repositories:lz4.BUILD.bazel",
        sha256 = "537512904744b35e232912055ccf8ec66d768639ff3abe5788d90d792ec5f48b",
        strip_prefix = "lz4-1.10.0",
        urls = ["https://github.com/lz4/lz4/archive/refs/tags/v1.10.0.tar.gz"],
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
        sha256 = "b93c667d1b69265cdb4d9f30ec21f8facbbe8b307cf34c0b9942834c6d4fdbe2",
        strip_prefix = "eigen-3.4.1",
        urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.4.1/eigen-3.4.1.tar.gz"],
    )
