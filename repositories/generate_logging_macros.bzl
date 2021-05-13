""" Implements a rule to generate logging macros for rcutils.
"""

def _logging_macros_impl(ctx):
    output = ctx.actions.declare_file(ctx.attr.output)
    template = ctx.attr.template
    ctx.actions.run(
        inputs = template.files,
        outputs = [output],
        executable = ctx.executable._generator,
        arguments = [output.path, template.files.to_list()[0].path],
    )
    return [DefaultInfo(files = depset([output]))]

logging_macros = rule(
    attrs = {
        "output": attr.string(
            mandatory = True,
        ),
        "template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "_generator": attr.label(
            default = Label("@ros2_rcutils//:generate_logging_macros"),
            executable = True,
            cfg = "exec",
        ),
    },
    implementation = _logging_macros_impl,
)
