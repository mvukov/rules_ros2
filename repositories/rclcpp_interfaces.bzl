""" Implements a rule for auto-generation of rclcpp interface traits and
    getters.
"""

def _get_stem(src):
    return src.basename[:-len(src.extension) - 1]

def _generate(ctx, interface_name, template, output):
    generator_args = ctx.actions.args()
    generator_args.add(interface_name)
    generator_args.add(output)
    generator_args.add(template)

    ctx.actions.run(
        inputs = [template],
        outputs = [output],
        executable = ctx.executable.generator,
        arguments = [generator_args],
        mnemonic = "Ros2RclcppInterfaces",
        progress_message = "Generating rclcpp interfaces for %{label}",
    )

def _rclcpp_interfaces_impl(ctx):
    interfaces = ctx.attr.interfaces[DefaultInfo].files.to_list()
    prefix_path = ctx.attr.prefix_path
    traits_template = ctx.file.traits_template
    getter_template = ctx.file.getter_template
    generated_files = []
    for interface in interfaces:
        interface_name = _get_stem(interface)
        interface_traits = ctx.actions.declare_file(
            "{}/{}_traits.hpp".format(
                prefix_path,
                interface_name,
            ),
        )
        interface_getter = ctx.actions.declare_file(
            "{}/get_{}.hpp".format(prefix_path, interface_name),
        )

        _generate(
            ctx,
            interface_name,
            traits_template,
            interface_traits,
        )
        _generate(
            ctx,
            interface_name,
            getter_template,
            interface_getter,
        )

        generated_files.append(interface_traits)
        generated_files.append(interface_getter)

    return [DefaultInfo(files = depset(generated_files))]

rclcpp_interfaces = rule(
    attrs = {
        "generator": attr.label(
            mandatory = True,
            executable = True,
            cfg = "exec",
        ),
        "getter_template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "interfaces": attr.label(
            mandatory = True,
        ),
        "prefix_path": attr.string(
            mandatory = True,
        ),
        "traits_template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
    },
    implementation = _rclcpp_interfaces_impl,
)
