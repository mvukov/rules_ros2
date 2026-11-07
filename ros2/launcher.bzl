""" Defines the launcher shared by all ros2_* binary and test macros.

A ros2_* target is a two-stage exec chain:

    <name>                          hermetic_launcher stub (tiny native binary)
      -> //ros2/launcher:ros2_exec  sets AMENT_PREFIX_PATH, ROS_HOME, ROS_LOG_DIR
        -> <name>_impl              the actual cc_binary / py_binary / rust_test / ...

The stub resolves every embedded runfiles path to an absolute path and exports
RUNFILES_DIR to its child, so starting a ROS 2 target needs neither /bin/sh nor
a Python interpreter (e.g. in distroless OCI images).
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load(
    "@com_github_mvukov_rules_ros2//ros2:ament.bzl",
    "Ros2AmentSetupInfo",
    "ros2_ament_setup",
)
load("@hermetic_launcher//launcher:lib.bzl", "launcher")
load("@rules_python//python:defs.bzl", "PyInfo")

def _to_rlocation_path(ctx, short_path):
    if short_path.startswith("../"):
        return short_path[3:]
    return paths.join(ctx.workspace_name, short_path)

def _ros2_launcher_impl(ctx):
    output = ctx.actions.declare_file(ctx.label.name)

    runfiles = ctx.runfiles(
        files = [ctx.executable._ros2_exec, ctx.executable.entrypoint] +
                ctx.files.runfile_args + ctx.files.data,
    )
    transitive_runfiles = [
        ctx.attr._ros2_exec[DefaultInfo].default_runfiles,
        ctx.attr.entrypoint[DefaultInfo].default_runfiles,
    ] + [dep[DefaultInfo].default_runfiles for dep in ctx.attr.runfile_args + ctx.attr.data]

    # argv: ros2_exec <ament_prefix_path|-> <entrypoint> [runfile_args...] [runtime args...]
    builder = launcher.entrypoint(ctx.executable._ros2_exec)
    ament_prefix_path = None
    if ctx.attr.ament_setup != None:
        ament_prefix_path = ctx.attr.ament_setup[Ros2AmentSetupInfo].ament_prefix_path
        transitive_runfiles.append(ctx.attr.ament_setup[DefaultInfo].default_runfiles)
    if ament_prefix_path == None:
        builder.embedded_args("-")
    else:
        builder.raw_transformed_args(_to_rlocation_path(ctx, ament_prefix_path))
    builder.runfiles(ctx.executable.entrypoint, *ctx.files.runfile_args)
    builder.compile(ctx, output_file = output)

    env = {
        key: ctx.expand_location(value, ctx.attr.data)
        for key, value in ctx.attr.env.items()
    }
    providers = [
        DefaultInfo(
            executable = output,
            files = depset([output]),
            runfiles = runfiles.merge_all(transitive_runfiles),
        ),
        RunEnvironmentInfo(environment = env),
        coverage_common.instrumented_files_info(
            ctx,
            dependency_attributes = ["entrypoint", "data"],
        ),
    ]

    # Keep e.g. py_library deps on a ros2_py_binary working.
    if PyInfo in ctx.attr.entrypoint:
        providers.append(ctx.attr.entrypoint[PyInfo])
    return providers

_COMMON_ATTRS = {
    "ament_setup": attr.label(
        doc = "If set, AMENT_PREFIX_PATH points to this ament index at runtime; otherwise it is unset.",
        providers = [Ros2AmentSetupInfo],
    ),
    "data": attr.label_list(allow_files = True),
    "entrypoint": attr.label(
        doc = "The executable to run.",
        mandatory = True,
        executable = True,
        cfg = "target",
    ),
    "env": attr.string_dict(
        doc = "Environment variables set when the target is run. Subject to $(location) expansion.",
    ),
    "runfile_args": attr.label_list(
        doc = "Files whose absolute runfiles paths are passed as the leading arguments of `entrypoint`.",
        allow_files = True,
    ),
    "_ros2_exec": attr.label(
        default = "@com_github_mvukov_rules_ros2//ros2/launcher:ros2_exec",
        executable = True,
        cfg = "target",
    ),
}

_TOOLCHAINS = [
    launcher.finalizer_toolchain_type,
    launcher.template_toolchain_type,
]

ros2_launcher_binary = rule(
    doc = "A native launcher for a ROS 2 binary. Use via the `ros2_launcher` macro.",
    attrs = _COMMON_ATTRS,
    executable = True,
    implementation = _ros2_launcher_impl,
    toolchains = _TOOLCHAINS,
)

ros2_launcher_test = rule(
    doc = "A native launcher for a ROS 2 test. Use via the `ros2_launcher` macro.",
    attrs = _COMMON_ATTRS,
    implementation = _ros2_launcher_impl,
    test = True,
    toolchains = _TOOLCHAINS,
)

def ros2_launcher(name, entrypoint, is_test, ament_setup_deps = None, idl_deps = None, **kwargs):
    """Defines a native launcher that sets up the ROS 2 environment and exec()s `entrypoint`.

    Used by all ros2_* binary and test macros (ros2_cpp_binary, ros2_py_test, ros2_launch, ...).

    For binaries the launcher sets AMENT_PREFIX_PATH (or unsets it when `ament_setup_deps` is None).
    For tests it additionally defaults ROS_HOME and ROS_LOG_DIR to $TEST_UNDECLARED_OUTPUTS_DIR
    (or $TEST_TMPDIR).

    Args:
        name: A unique target name.
        entrypoint: The executable target to wrap.
        is_test: If true, defines a test target, otherwise a binary target. Independent of
            `testonly`, which only restricts who may depend on the launcher.
        ament_setup_deps: Dependencies for ament setup. If None, ament setup is skipped.
        idl_deps: Additional IDL dependencies used as runtime plugins.
        **kwargs: Forwarded to the launcher rule (e.g. `args`, `data`, `env`, `size`, `tags`,
            `timeout`, `visibility`).
    """
    testonly = kwargs.pop("testonly", is_test)
    ament_setup = None
    if ament_setup_deps != None:
        ament_setup = name + "_ament_setup"
        ros2_ament_setup(
            name = ament_setup,
            deps = ament_setup_deps,
            idl_deps = idl_deps,
            tags = ["manual"],
            testonly = testonly,
        )
    rule_impl = ros2_launcher_test if is_test else ros2_launcher_binary
    rule_impl(
        name = name,
        entrypoint = entrypoint,
        ament_setup = ament_setup,
        testonly = testonly,
        **kwargs
    )
