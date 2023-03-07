"""Rule for simple expansion of template files.

This performs a simple search over the template file for the keys in
substitutions, and replaces them with the corresponding values.

Taken from https://github.com/bazelbuild/bazel-skylib/issues/191

Typical usage:
  expand_template(
      name = "ExpandMyTemplate",
      template = "my.template",
      out = "my.txt",
      substitutions = {
        "$VAR1": "foo",
        "$VAR2": "bar",
      }
  )

Args:
  name: The name of the rule.
  template: The template file to expand
  out: The destination of the expanded file
  substitutions: A dictionary mapping strings to their substitutions
  is_executable: A boolean indicating whether the output file should be executable
"""

def expand_template_impl(ctx, template, output, substitutions, is_executable):
    ctx.actions.expand_template(
        template = template,
        output = output,
        substitutions = {
            k: ctx.expand_location(v, ctx.attr.data)
            for k, v in substitutions.items()
        },
        is_executable = is_executable,
    )

def _expand_template_impl(ctx):
    expand_template_impl(
        ctx,
        ctx.file.template,
        ctx.outputs.out,
        ctx.attr.substitutions,
        ctx.attr.is_executable,
    )

expand_template = rule(
    implementation = _expand_template_impl,
    attrs = {
        "template": attr.label(mandatory = True, allow_single_file = True),
        "substitutions": attr.string_dict(mandatory = True),
        "out": attr.output(mandatory = True),
        "is_executable": attr.bool(default = False, mandatory = False),
        "data": attr.label_list(allow_files = True),
    },
)
