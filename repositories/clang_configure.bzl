_LIBCLANG_CANDIDATE_TEMPLATE_PATHS = [
    "/usr/lib/llvm-{0}/lib/libclang.so.1",
    "/usr/lib/llvm-{0}/lib/libclang-{0}.so.1",
]

def _clang_configure_impl(repository_ctx):
    clang_bin_path = repository_ctx.which("clang")
    if clang_bin_path == None:
        fail("Failed to find clang executable")

    repository_ctx.symlink(clang_bin_path, "clang")

    result = repository_ctx.execute([clang_bin_path, "--version"])
    if result.return_code != 0:
        fail("Failed to get clang version")
    clang_version = result.stdout.split(" ")[3].split(".")[0]

    libclang_path = None
    for candidate_tpl_path in _LIBCLANG_CANDIDATE_TEMPLATE_PATHS:
        candidate_path = candidate_tpl_path.format(clang_version)
        if not repository_ctx.path(candidate_path).exists:
            continue

        result = repository_ctx.execute(["realpath", candidate_path])
        if result.return_code != 0:
            continue

        libclang_path = result.stdout.strip()
        break

    if libclang_path == None:
        fail("Failed to fetch libclang path, candidates: " + ", ".join([c.format(clang_version) for c in _LIBCLANG_CANDIDATE_TEMPLATE_PATHS]))

    repository_ctx.symlink(libclang_path, "libclang.so")

    repository_ctx.file("BUILD.bazel", """\
load("@rules_cc//cc:defs.bzl", "cc_import")

exports_files([
    "clang",
])

cc_import(
    name = "libclang",
    shared_library = "libclang.so",
    visibility = ["//visibility:public"],
)
""")

clang_configure = repository_rule(
    implementation = _clang_configure_impl,
)
