""" Implements a rule to generate logging macros.
"""

def _logging_macros_impl(ctx):
    output = ctx.actions.declare_file(ctx.attr.output)
    template = ctx.file.template
    args = ctx.actions.args()
    args.add(output)
    args.add(template)
    ctx.actions.run(
        inputs = [template],
        outputs = [output],
        executable = ctx.executable.generator,
        arguments = [args],
        mnemonic = "Ros2LoggingMacros",
        progress_message = "Generating logging macros for %{label}",
    )
    return [DefaultInfo(files = depset([output]))]

logging_macros = rule(
    attrs = {
        "generator": attr.label(
            mandatory = True,
            executable = True,
            cfg = "exec",
        ),
        "output": attr.string(
            mandatory = True,
        ),
        "template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
    },
    implementation = _logging_macros_impl,
)
