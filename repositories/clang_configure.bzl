def _clang_configure_impl(repository_ctx):
    clang_bin_path = repository_ctx.which("clang")
    if clang_bin_path == None:
        fail("Failed to find clang executable")

    repository_ctx.symlink(clang_bin_path, "clang")

    os_name = repository_ctx.os.name.lower()
    is_mac = "mac" in os_name or "darwin" in os_name
    
    libclang_path = None
    lib_ext = "so"

    if is_mac:
        lib_ext = "dylib"
        # Try to find libclang on macOS
        # 1. Check xcode-select path
        result = repository_ctx.execute(["xcode-select", "-p"])
        if result.return_code == 0:
            xcode_dir = result.stdout.strip()
            # Standard locations inside Xcode/CLT
            candidates = [
                "{}/usr/lib/libclang.dylib".format(xcode_dir),
                "{}/Toolchains/XcodeDefault.xctoolchain/usr/lib/libclang.dylib".format(xcode_dir),
                # If xcode_dir is /Library/Developer/CommandLineTools
                "{}/Library/Frameworks/libclang.dylib".format(xcode_dir), 
            ]
            for candidate in candidates:
                if repository_ctx.execute(["test", "-f", candidate]).return_code == 0:
                    libclang_path = candidate
                    break
        
        # 2. Fallback to Homebrew/system paths if not found
        if libclang_path == None:
            candidates = [
                "/usr/lib/libclang.dylib",
                "/opt/homebrew/opt/llvm/lib/libclang.dylib",
                "/usr/local/opt/llvm/lib/libclang.dylib",
            ]
            for candidate in candidates:
                if repository_ctx.execute(["test", "-f", candidate]).return_code == 0:
                    libclang_path = candidate
                    break
                    
        if libclang_path == None:
             # Last resort: try to ask clang where it is installed and look relative to that
             # This is tricky with Apple clang wrappers
             pass

    else:
        # Linux logic
        result = repository_ctx.execute([clang_bin_path, "--version"])
        if result.return_code != 0:
            fail("Failed to get clang version")
        
        # Output format varies. Assume typical "clang version X.Y.Z ..."
        # or "Ubuntu clang version ..."
        parts = result.stdout.split(" ")
        clang_version = None
        for i, part in enumerate(parts):
            if part == "version" and i + 1 < len(parts):
                clang_version = parts[i+1].split(".")[0]
                break
        
        if clang_version:
             # Try ubuntu style path
             candidate = "/usr/lib/llvm-{}/lib/libclang.so.1".format(clang_version)
             if repository_ctx.execute(["test", "-f", candidate]).return_code == 0:
                 libclang_path = candidate
             else:
                 # Try generic /usr/lib
                 candidate = "/usr/lib/libclang.so"
                 if repository_ctx.execute(["test", "-f", candidate]).return_code == 0:
                     libclang_path = candidate
                 else:
                     # Try looking in the directory of clang binary
                     # clang_bin_path might be /usr/bin/clang -> /usr/lib/libclang.so
                     pass

    if libclang_path == None:
        # Final fallback for any OS: assume we can't find it easily and fail with a clear message
        # or try to use 'locate' or 'find'? (Too slow)
        fail("Failed to find libclang.{} on this system. Please ensure LLVM/Clang is installed.".format(lib_ext))

    # Resolve symlink to get real path if needed, though we just symlink it to our repo
    repository_ctx.symlink(libclang_path, "libclang.{}".format(lib_ext))

    repository_ctx.file("BUILD.bazel", """
exports_files([
    "clang",
])

cc_import(
    name = "libclang",
    shared_library = "libclang.{}",
    visibility = ["//visibility:public"],
)
""".format(lib_ext))

clang_configure = repository_rule(
    implementation = _clang_configure_impl,
)
