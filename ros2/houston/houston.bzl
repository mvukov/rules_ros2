load("@rules_python//python:defs.bzl", "PyInfo")

HoustonDeploymentInfo = provider(
    "Provides info for deployment generation",
    fields = [
        "launch_files",
        "launch_deps",
        "nodes",
        "parameters",
    ],
)

def _ros2_deployment_impl(ctx):
    return [
        # DefaultInfo(files = depset(ctx.files.srcs)),
        HoustonDeploymentInfo(
            launch_files = depset(
                direct = [] if ctx.attr.launch_file == None else [ctx.attr.launch_file],
                transitive = [
                    dep[HoustonDeploymentInfo].launch_files
                    for dep in ctx.attr.deps
                ],
            ),
            launch_deps = depset(
                direct = ctx.attr.launch_deps,
                transitive = [
                    dep[HoustonDeploymentInfo].launch_deps
                    for dep in ctx.attr.deps
                ],
            ),
            nodes = depset(
                direct = ctx.attr.nodes,
                transitive = [
                    dep[HoustonDeploymentInfo].nodes
                    for dep in ctx.attr.deps
                ],
            ),
            parameters = depset(
                direct = ctx.attr.parameters,
                transitive = [
                    dep[HoustonDeploymentInfo].parameters
                    for dep in ctx.attr.deps
                ],
            ),
        ),
    ]

ros2_deployment = rule(
    attrs = {
        "launch_file": attr.label(
            allow_files = [".py"],
        ),
        "launch_deps": attr.label_list(
            providers = [PyInfo],
            cfg = "exec",
        ),
        "nodes": attr.label_list(
            cfg = "target",
        ),
        "parameters": attr.label_list(
            allow_files = [".yaml", ".yml"],
        ),
        "deps": attr.label_list(
            providers = [HoustonDeploymentInfo],
        ),
    },
    implementation = _ros2_deployment_impl,
    provides = [HoustonDeploymentInfo],
)

def _merge_houston_deployment_infos(infos):
    return struct(
        launch_files = depset(transitive = [info.launch_files for info in infos]),
        launch_deps = depset(transitive = [info.launch_deps for info in infos]),
        nodes = depset(transitive = [info.nodes for info in infos]),
        parameters = depset(transitive = [info.parameters for info in infos]),
    )

def _format_launch_file(launch_file):
    return [ff.path for ff in launch_file[DefaultInfo].files.to_list()]

def _get_node_root_paths(node):
    return [ff.short_path for ff in node[DefaultInfo].files.to_list()]

SH_TOOLCHAIN = "@rules_shell//shell:toolchain_type"

def _ros2_launch_impl(ctx):
    deps = [dep[HoustonDeploymentInfo] for dep in ctx.attr.deps]
    merged_deps = _merge_houston_deployment_infos(deps)

    merged_params_file = ctx.actions.declare_file(
        "{}_merged_params.yaml".format(ctx.attr.name),
    )
    ground_control_config_file = ctx.actions.declare_file(
        "{}_ground_control.toml".format(ctx.attr.name),
    )

    generator_args = ctx.actions.args()
    generator_args.add_all(
        "--deployment_specs",
        merged_deps.launch_files,
        map_each = _format_launch_file,
    )
    generator_args.add_all(
        "--executable_paths",
        merged_deps.nodes,
        map_each = _get_node_root_paths,
    )
    generator_args.add(merged_params_file.path, format = "--merged_params_exec_path=%s")
    generator_args.add(merged_params_file.short_path, format = "--merged_params_root_path=%s")
    generator_args.add(ground_control_config_file, format = "--groundcontrol_config=%s")

    ctx.actions.run(
        inputs = depset(
            transitive =
                [target[DefaultInfo].files for target in merged_deps.launch_files.to_list()] +
                [target[DefaultInfo].files for target in merged_deps.parameters.to_list()] +
                [target[DefaultInfo].files for target in merged_deps.nodes.to_list()],
        ),
        outputs = [merged_params_file, ground_control_config_file],
        executable = ctx.executable._generator,
        arguments = [generator_args],
        mnemonic = "HoustonGroundControlConfig",
        progress_message = "Generating deployment config files for %{label}",
    )

    executable = ctx.actions.declare_file(
        "{}_launch.sh".format(ctx.attr.name),
    )

    ctx.actions.expand_template(
        template = ctx.file._ground_control_config_template,
        output = executable,
        substitutions = {
            "{{bash_bin}}": ctx.toolchains[SH_TOOLCHAIN].path,
            "{{ground_control_bin}}": ctx.executable._ground_control.short_path,
            "{{ground_control_config}}": ground_control_config_file.short_path,
        },
        is_executable = True,
    )

    nodes = merged_deps.nodes.to_list()
    runfiles = ctx.runfiles(
        files = [
            ctx.executable._ground_control,
            ground_control_config_file,
            merged_params_file,
        ],
        transitive_files = depset(
            transitive = [node[DefaultInfo].files for node in nodes],
        ),
    )
    for node in nodes:
        runfiles = runfiles.merge(node[DefaultInfo].default_runfiles)

    return [
        DefaultInfo(
            files = depset([merged_params_file, ground_control_config_file]),
            executable = executable,
            runfiles = runfiles,
        ),
    ]

ros2_launch = rule(
    attrs = {
        "deps": attr.label_list(
            providers = [HoustonDeploymentInfo],
        ),
        "_generator": attr.label(
            default = Label("@com_github_mvukov_rules_ros2//ros2/houston:ground_control_config_generator"),
            executable = True,
            cfg = "exec",
        ),
        "_ground_control": attr.label(
            default = Label("@com_github_mvukov_rules_ros2//third_party/ground_control"),
            executable = True,
            cfg = "target",
        ),
        "_ground_control_config_template": attr.label(
            default = Label("@com_github_mvukov_rules_ros2//ros2/houston:launch.sh.tpl"),
            allow_single_file = True,
        ),
    },
    executable = True,
    implementation = _ros2_launch_impl,
    toolchains = [SH_TOOLCHAIN],
)
