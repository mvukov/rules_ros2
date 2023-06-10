""" Defines a rule for transforming xacro files to a UDRF.
"""

def _xacro_impl(ctx):
    output = ctx.actions.declare_file(ctx.attr.name + "/model.urdf")
    srcs = ctx.files.srcs
    main = ctx.files.main

    if len(main) == 0:
        main = [srcs[0]]
    elif main not in srcs:
        srcs = srcs + main

    args = ctx.actions.args()
    args.add_all(ctx.attr.args)
    args.add_all(main)
    args.add(output, format = "--output=%s")

    ctx.actions.run(
        inputs = srcs,
        outputs = [output],
        executable = ctx.executable._xacro,
        arguments = [args],
        mnemonic = "Ros2Xacro",
        progress_message = "Generating URDF for %{label}",
    )
    return [
        DefaultInfo(
            files = depset([output]),
        ),
    ]

xacro = rule(
    attrs = {
        "srcs": attr.label_list(
            allow_files = [".xacro", ".yaml"],
            mandatory = True,
        ),
        "main": attr.label(
            allow_files = [".xacro"],
        ),
        "args": attr.string_list(),
        "_xacro": attr.label(
            default = Label("@ros2_xacro//:app"),
            executable = True,
            cfg = "exec",
        ),
    },
    implementation = _xacro_impl,
)
