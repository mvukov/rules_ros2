def _clang_configure_impl(repository_ctx):
    clang_bin_path = repository_ctx.which("clang")
    if clang_bin_path == None:
        fail("Failed to find clang executable:\n{}".format(result.stderr))

    repository_ctx.symlink(clang_bin_path, "clang")

    result = repository_ctx.execute([clang_bin_path, "--version"])
    if result.return_code != 0:
        fail("Failed to get clang version")
    clang_version = result.stdout.split(" ")[3].split(".")[0]

    result = repository_ctx.execute(["realpath", "/usr/lib/llvm-{}/lib/libclang.so.1".format(clang_version)])
    if result.return_code != 0:
        fail("Failed to fetch libclang_path:\n{}".format(result.stderr))
    libclang_path = result.stdout.strip()
    repository_ctx.symlink(libclang_path, "libclang.so")

    repository_ctx.file("BUILD.bazel", """\
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
