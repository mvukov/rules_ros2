HoustonDeploymentInfo = provider(
    "Provides info for deployment generation",
    fields = [
        "launch_files",
        # Serves to check that all process definitions exist within a set of known binaries.
        "executable_files",
        "params_files",
    ],
)

def _ros2_deployment_impl(ctx):
    direct_files_depsets = []
    for node in ctx.attr.nodes:
        direct_files_depsets.append(node[DefaultInfo].files)

    transitive_files_depsets = []
    for target in ctx.attr.data:
        transitive_files_depsets.append(target[DefaultInfo].files)
    for dep in ctx.attr.deps:
        transitive_files_depsets.append(dep[DefaultInfo].files)

    runfiles = ctx.runfiles(
        transitive_files = depset(transitive = transitive_files_depsets),
    ).merge_all(
        [target[DefaultInfo].default_runfiles for target in ctx.attr.data] +
        [dep[DefaultInfo].default_runfiles for dep in ctx.attr.deps] +
        [node[DefaultInfo].default_runfiles for node in ctx.attr.nodes],
    )

    launch_file = []
    if ctx.attr.launch_file != None:
        launch_file = [ctx.attr.launch_file[DefaultInfo].files]

    params_files = []
    if ctx.attr.parameters != None:
        params_files = [param[DefaultInfo].files for param in ctx.attr.parameters]

    executable_files = []
    if ctx.attr.nodes != None:
        for node in ctx.attr.nodes:
            if node[DefaultInfo].files_to_run == None or node[DefaultInfo].files_to_run.executable == None:
                fail("{} is not an executable")

        # TODO(mvukov) The last thing I know is that for py_binary both the interpreter
        # and the shim are in files, to be double-checked.
        executable_files = [node[DefaultInfo].files for node in ctx.attr.nodes]

    return [
        DefaultInfo(
            files = depset(transitive = direct_files_depsets),
            runfiles = runfiles,
        ),
        HoustonDeploymentInfo(
            launch_files = depset(
                transitive = launch_file + [
                    dep[HoustonDeploymentInfo].launch_files
                    for dep in ctx.attr.deps
                ],
            ),
            executable_files = depset(
                transitive = executable_files + [
                    dep[HoustonDeploymentInfo].executable_files
                    for dep in ctx.attr.deps
                ],
            ),
            params_files = depset(
                transitive = params_files + [
                    dep[HoustonDeploymentInfo].params_files
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
        "nodes": attr.label_list(
            cfg = "target",
        ),
        "parameters": attr.label_list(
            allow_files = [".yaml", ".yml"],
        ),
        "deps": attr.label_list(
            providers = [HoustonDeploymentInfo],
        ),
        "data": attr.label_list(
            allow_files = True,
        ),
    },
    implementation = _ros2_deployment_impl,
    provides = [DefaultInfo, HoustonDeploymentInfo],
)

def _merge_houston_deployment_infos(infos):
    return HoustonDeploymentInfo(
        launch_files = depset(transitive = [info.launch_files for info in infos]),
        executable_files = depset(transitive = [info.executable_files for info in infos]),
        params_files = depset(transitive = [info.params_files for info in infos]),
    )

def _format_launch_file(launch_file):
    return launch_file.path

def _get_root_path(file):
    return file.short_path

SH_TOOLCHAIN = "@rules_shell//shell:toolchain_type"

def _ros2_launch_impl(ctx):
    deps_infos = [dep[HoustonDeploymentInfo] for dep in ctx.attr.deps]
    merged_infos = _merge_houston_deployment_infos(deps_infos)

    merged_params_file = ctx.actions.declare_file(
        "{}_merged_params.yaml".format(ctx.attr.name),
    )
    ground_control_config_file = ctx.actions.declare_file(
        "{}_ground_control.toml".format(ctx.attr.name),
    )

    generator_args = ctx.actions.args()
    generator_args.add_all(
        "--deployment_specs",
        merged_infos.launch_files,
        map_each = _format_launch_file,
    )
    generator_args.add_all(
        "--executable_paths",
        merged_infos.executable_files,
        map_each = _get_root_path,
    )
    generator_args.add(merged_params_file.path, format = "--merged_params_exec_path=%s")
    generator_args.add(merged_params_file.short_path, format = "--merged_params_root_path=%s")
    generator_args.add(ground_control_config_file, format = "--groundcontrol_config=%s")

    ctx.actions.run(
        inputs = depset(transitive = [merged_infos.launch_files, merged_infos.params_files]),
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

    runfiles = ctx.runfiles(
        files = [
            ctx.executable._ground_control,
            ground_control_config_file,
            merged_params_file,
        ],
        transitive_files = depset(transitive = [dep[DefaultInfo].files for dep in ctx.attr.deps]),
    ).merge_all(
        [dep[DefaultInfo].default_runfiles for dep in ctx.attr.deps],
    )

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
            default = Label("@com_github_mvukov_rules_ros2//ros2/houston:generate_config"),
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
