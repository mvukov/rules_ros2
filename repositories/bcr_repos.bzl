load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def bcr_repositories():
    maybe(
        http_archive,
        name = "rules_cc",
        urls = ["https://github.com/bazelbuild/rules_cc/releases/download/0.2.8/rules_cc-0.2.8.tar.gz"],
        integrity = "sha256-IH6gc90gpwX56LxawC9SA+liH8Zyd0uxoJNa76t66/o=",
        strip_prefix = "rules_cc-0.2.8",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/rules_cc/0.2.8/MODULE.bazel"],
        remote_patches = {"https://bcr.bazel.build/modules/rules_cc/0.2.8/patches/module_dot_bazel_version.patch": "sha256-cr93LzUmi+/z+Ppbc6LS4iNyt6ZnB6z0kG3jC3+3tgE="},
        remote_patch_strip = 1,
    )

    maybe(
        http_archive,
        name = "platforms",
        urls = ["https://github.com/bazelbuild/platforms/releases/download/1.0.0/platforms-1.0.0.tar.gz"],
        integrity = "sha256-M4TrHDB2JwT7445EAgThFBVAhsj8iowuPihEECjAGag=",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/platforms/1.0.0/MODULE.bazel"],
    )

    maybe(
        http_archive,
        name = "apple_support",
        urls = ["https://github.com/bazelbuild/apple_support/releases/download/1.24.1/apple_support.1.24.1.tar.gz"],
        integrity = "sha256-onDwNAB6thEWRdJ6jtiLR84qdbYIr2M+7KiJy14Hrg0=",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/apple_support/1.24.1/MODULE.bazel"],
        remote_patches = {"https://bcr.bazel.build/modules/apple_support/1.24.1/patches/module_dot_bazel_version.patch": "sha256-4dfsgoUNMKGhrxm42FNcuZnv5KP3Ddq0UlbYnV4MplU="},
        remote_patch_strip = 1,
    )

    maybe(
        http_archive,
        name = "llvm-project",
        urls = ["https://github.com/llvm/llvm-project/releases/download/llvmorg-17.0.3/llvm-project-17.0.3.src.tar.xz"],
        integrity = "sha256-vloeRNZPMGu0T859NuOzmTaU6OYSKyNIYIkGKDwXbbg=",
        strip_prefix = "llvm-project-17.0.3.src",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/MODULE.bazel"],
        remote_patches = {
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0001-Add-Bazel-files-to-.gitignore.patch": "sha256-JC5X1fmrDR0auseyjZxlo9zyb242FgjaXPqkVewh7uc=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0002-Add-LLVM-Bazel-overlay-files.patch": "sha256-w7Lz/Km3RGzKCW4IwFI61p8nNjRMMcT8T0uK4Y1aans=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0003-Add-MODULE.bazel.patch": "sha256-1vn6wJpku62Xl/I7VPUUwhXlrm4YOL/4sTSMQ4IDn0g=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0004-Add-BUILD.bazel.patch": "sha256-CEeI0zIB9Q2Gmg+bHl9+xlOMPCektFA1CuiDVJUuJvY=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0005-Add-Bazel-LLVM-targets.patch": "sha256-2MqswOFWJWqE4Dh/iWsinsVgEatqT6DtE9f4gx8+SgA=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0006-Add-LLVM-version-vars.patch": "sha256-pMrccxE0q/iD6T97t1x9QFCdndi9zUrhF6mY/o+TXt8=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0008-Correct-builtin-headers-prefix.patch": "sha256-jlbVGyo5Hy+HWhs6GkLKKltOdlKC42DUqzwm6X0oePw=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0009-Fix-an-operator-overload-for-GCC-8.3.patch": "sha256-H6g6djsyxbtUISZFUPJJwuw2zqPHs6dqDQF7I5Gkwzs=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0010-Use-TEST_TMPDIR-on-Unix-when-it-is-set.patch": "sha256-ksYPHSFXRpLuLo8n4XDsP2RT+2zoFZvIiXU+STwHHBs=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0011-Use-Windows-assembly-files-for-BLAKE3-on-Windows.patch": "sha256-/L/8cQoenbvTz6Y0l3VEzCCIPnGewqz3NyqFWb2M+tI=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0012-Add-scope-resolution-operators-for-MSVC-2019.patch": "sha256-PsrLVgyyXWW/+Xe49INQPxObNVJk+zWjCgm3BOL8Yfk=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0013-Disable-zlib-zstd-mpfr-and-pfm.patch": "sha256-XbSq5b7eS9Kc9weOGiWDVJA5L43E/4jMbvJaYMCvxrA=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0014-Incompatible-Disable-Empty-Glob.patch": "sha256-n/W3+RW/LeitJ2iKy+qoFEsLSSAirsMd00WN2/CGLc4=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0015-incompatible_autoload_externally.patch": "sha256-yJZ9rn4A1vHJjTBztnMXJkQ70M5Ft3T0rdz0IkHLPtU=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0016-extend-clang-test-timeout.patch": "sha256-47DEQpj8HBSa+/TImW+5JCeuQeRkm5NMpJWZG3hSuFU=",
            "https://bcr.bazel.build/modules/llvm-project/17.0.3.bcr.4/patches/0017-Avoid-bazel-tools-constraints.patch": "sha256-PfpWEMLxZnWjCc6U/4ETnk3ztxJOygKlr6PweCt5KJo="
        },
        remote_patch_strip = 1,
    )

    maybe(
        http_archive,
        name = "abseil-cpp",
        urls = ["https://github.com/abseil/abseil-cpp/releases/download/20250127.1/abseil-cpp-20250127.1.tar.gz"],
        integrity = "sha256-s5ZAH9KeLmecrOd4Z0gdOIyAdnHcKsxgKgJZ7rebeBE=",
        strip_prefix = "abseil-cpp-20250127.1",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/abseil-cpp/20250127.1/MODULE.bazel"],
    )

    maybe(
        http_archive,
        name = "re2",
        urls = ["https://github.com/google/re2/releases/download/2024-07-02/re2-2024-07-02.zip"],
        integrity = "sha256-qDX+Vfvc2OgPOFhKsi0IQGYsZ/L+s2vWeUAtqWQdxx4=",
        strip_prefix = "re2-2024-07-02",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/re2/2024-07-02.bcr.1/MODULE.bazel"],
        remote_patches = {"https://bcr.bazel.build/modules/re2/2024-07-02.bcr.1/patches/module_dot_bazel.patch": "sha256-qacD5PFevrt/ZkvLjKCocyCICqh/pVrb5J1Ho+tTsoc="},
    )

    maybe(
        http_archive,
        name = "protobuf",
        urls = ["https://github.com/protocolbuffers/protobuf/releases/download/v29.0/protobuf-29.0.zip"],
        integrity = "sha256-LkQtIYOeyduv2kzJCDI5qgTnj8nCffpZtTdOloBQzSI=",
        strip_prefix = "protobuf-29.0",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/protobuf/29.0/MODULE.bazel"],
    )

    maybe(
        http_archive,
        name = "nlohmann_json",
        urls = ["https://github.com/nlohmann/json/releases/download/v3.11.3/include.zip"],
        integrity = "sha256-oiRh0TEZrFx48gXT3x2xNAPljOG7F5TtyTE2dzE/Sp0=",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/nlohmann_json/3.11.3/MODULE.bazel"],
        remote_patches = {"https://bcr.bazel.build/modules/nlohmann_json/3.11.3/patches/module_dot_bazel.patch": "sha256-OmeSCp1IqWbHGPJs0v5taUiPLEsI9KEJPLsnPpKB/B8="},
    )

    maybe(
        http_archive,
        name = "stardoc",
        urls = ["https://github.com/bazelbuild/stardoc/releases/download/0.7.2/stardoc-0.7.2.tar.gz"],
        integrity = "sha256-Dh7UqY8m5xh3a9ZNBT0CuzTZhXLM0D1ro1URKhIFcGs=",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/stardoc/0.7.2/MODULE.bazel"],
    )

    maybe(
        http_archive,
        name = "jsoncpp",
        urls = ["https://github.com/open-source-parsers/jsoncpp/archive/refs/tags/1.9.5.tar.gz"],
        integrity = "sha256-9AmFblkgwY0ML7hSduJO5gfSoJtefV8KNxNokDwnXaI=",
        strip_prefix = "jsoncpp-1.9.5",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/jsoncpp/1.9.5/MODULE.bazel"],
        remote_patches = {
            "https://bcr.bazel.build/modules/jsoncpp/1.9.5/patches/build_dot_bazel.patch": "sha256-Vj8diXSWps8I8h5cdEqBDYmKBA2ulvWxMZBEQlIgcpU=",
            "https://bcr.bazel.build/modules/jsoncpp/1.9.5/patches/module_dot_bazel.patch": "sha256-7RC7fS8N11vcyeDEaUZ05yBqr0YY7OzuzqaWz5W2XDo="
        },
        remote_patch_strip = 1,
    )

    maybe(
        http_archive,
        name = "rules_fuzzing",
        urls = ["https://github.com/bazelbuild/rules_fuzzing/releases/download/v0.5.2/rules_fuzzing-0.5.2.zip"],
        integrity = "sha256-5rwhm/rJ4fg7Mn3QkPcoqflz7pm5tdjloYSicy7whiM=",
        strip_prefix = "rules_fuzzing-0.5.2",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/rules_fuzzing/0.5.2/MODULE.bazel"],
        remote_patches = {"https://bcr.bazel.build/modules/rules_fuzzing/0.5.2/patches/module_dot_bazel.patch": "sha256-+S/1nXWYEzeyvZeC9Zpgmt6bmmStrSBG99b33+dTmXc="},
    )

    maybe(
        http_archive,
        name = "rules_java",
        urls = ["https://github.com/bazelbuild/rules_java/releases/download/8.14.0/rules_java-8.14.0.tar.gz"],
        integrity = "sha256-u+fZQ2DMntRgfsX9lJlf0exB6EJXAgtvCeZAVSgeyxI=",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/rules_java/8.14.0/MODULE.bazel"],
    )

    maybe(
        http_archive,
        name = "rules_jvm_external",
        urls = ["https://github.com/bazelbuild/rules_jvm_external/releases/download/6.3/rules_jvm_external-6.3.tar.gz"],
        integrity = "sha256-wYpp14S82FG+lYl8oOygtX3Ia7AuYkAvFXNt9EFg6wI=",
        strip_prefix = "rules_jvm_external-6.3",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/rules_jvm_external/6.3/MODULE.bazel"],
    )

    maybe(
        http_archive,
        name = "rules_kotlin",
        urls = ["https://github.com/bazelbuild/rules_kotlin/releases/download/v1.9.6/rules_kotlin-v1.9.6.tar.gz"],
        integrity = "sha256-O3cpdv7Hvc2h2EudObF2WJQkwEfrIXW+0JqsYw5Qr0M=",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/rules_kotlin/1.9.6/MODULE.bazel"],
        remote_patches = {"https://bcr.bazel.build/modules/rules_kotlin/1.9.6/patches/module_dot_bazel_version.patch": "sha256-DzcJ53CqDqD+AiboAl8Tq2/fKJRXn0g5O2g4UQfLrbE="},
        remote_patch_strip = 1,
    )

    maybe(
        http_archive,
        name = "rules_pkg",
        urls = ["https://github.com/bazelbuild/rules_pkg/releases/download/1.0.1/rules_pkg-1.0.1.tar.gz"],
        integrity = "sha256-0gyVGWDtd8t7NBwqWUiFNOSU1a0dMMSBjHNtV3cqn+8=",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/rules_pkg/1.0.1/MODULE.bazel"],
    )

    maybe(
        http_archive,
        name = "zlib",
        urls = ["https://github.com/madler/zlib/releases/download/v1.3.1/zlib-1.3.1.tar.gz"],
        integrity = "sha256-mpOyt9/ax3zrpaVYpYDnRmfdb+3kWFuR7vtg8Dty3yM=",
        strip_prefix = "zlib-1.3.1",
        remote_module_file_urls = ["https://bcr.bazel.build/modules/zlib/1.3.1.bcr.5/MODULE.bazel"],
        remote_patches = {
            "https://bcr.bazel.build/modules/zlib/1.3.1.bcr.5/patches/add_build_file.patch": "sha256-SdbiiqOKN9dcerx8E+mFC2Pd/Q2KuL67/3+50WxCJLc=",
            "https://bcr.bazel.build/modules/zlib/1.3.1.bcr.5/patches/module_dot_bazel.patch": "sha256-ln6iWXu370RclA0exBzU2YboB6sDIn76lsAzkNXWuvk="
        },
    )
