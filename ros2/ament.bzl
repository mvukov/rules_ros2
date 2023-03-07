""" Defines ament-related utilities.
"""

load("@bazel_skylib//lib:dicts.bzl", "dicts")
load("@bazel_skylib//lib:paths.bzl", "paths")
load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "Ros2InterfaceInfo",
    "cpp_generator_aspect",
    "idl_adapter_aspect",
)
load(
    "@com_github_mvukov_rules_ros2//ros2:plugin_aspects.bzl",
    "Ros2IdlPluginAspectInfo",
    "Ros2PluginCollectorAspectInfo",
    "ros2_idl_plugin_aspect",
    "ros2_plugin_collector_aspect",
)
load(
    "@com_github_mvukov_rules_ros2//third_party:expand_template.bzl",
    "expand_template_impl",
)

_AMENT_SETUP_MODULE = "ament_setup"

_PACKAGE_XML_TEMPLATE = """\
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="2">
  <name>{package_name}</name>
</package>
"""

_RESOURCE_INDEX_PATH = "share/ament_index/resource_index"
_PACKAGES_PATH = paths.join(_RESOURCE_INDEX_PATH, "packages")
_PACKAGE_XML = "package.xml"

def _write_package_xml(ctx, prefix_path, package_name):
    package_xml = ctx.actions.declare_file(
        paths.join(prefix_path, _PACKAGES_PATH, package_name, _PACKAGE_XML),
    )
    ctx.actions.write(package_xml, _PACKAGE_XML_TEMPLATE.format(package_name = package_name))
    return package_xml

def get_plugins_xml_path(plugin_package, plugin_target_name):
    """Returns prefix-path relative plugins XML file."""
    return paths.join(_PACKAGES_PATH, plugin_package, plugin_target_name + "_plugins.xml")

def _write_plugin_manifest(ctx, prefix_path, base_package, plugin_package, plugin_target_name):
    manifest = ctx.actions.declare_file(
        paths.join(
            prefix_path,
            _RESOURCE_INDEX_PATH,
            base_package + "__pluginlib__plugin",
            plugin_target_name + "_manifest",
        ),
    )
    ctx.actions.write(manifest, get_plugins_xml_path(plugin_package, plugin_target_name))
    return manifest

_PLUGINS_XML_TEMPLATE = """\
<library path="{library_path}">
{plugins}
</library>
"""

_PLUGIN_XML_TEMPLATE = """\
<class name="{class_name}" type="{class_type}" base_class_type="{base_class_type}">
<description>{class_type}</description>
</class>
"""

def _write_plugins_xml(
        ctx,
        prefix_path,
        plugin_package,
        plugin_target_name,
        types_to_bases_and_names):
    plugins_xml = ctx.actions.declare_file(
        paths.join(prefix_path, get_plugins_xml_path(plugin_package, plugin_target_name)),
    )

    plugins = [
        _PLUGIN_XML_TEMPLATE.format(
            class_name = base_class_type_and_name[1],
            class_type = class_type,
            base_class_type = base_class_type_and_name[0],
        )
        for class_type, base_class_type_and_name in types_to_bases_and_names.items()
    ]
    ctx.actions.write(plugins_xml, _PLUGINS_XML_TEMPLATE.format(
        library_path = plugin_target_name,
        plugins = "\n".join(plugins),
    ))
    return plugins_xml

def _get_package_name(class_name):
    return class_name.split("::")[0]

Ros2AmentSetupInfo = provider(
    "TBD",
    fields = [
        "ament_prefix_path",
    ],
)

def _ros2_ament_setup_rule_impl(ctx):
    plugins = depset(
        transitive = [
            dep[Ros2PluginCollectorAspectInfo].plugins
            for dep in ctx.attr.deps
        ],
    ).to_list()

    prefix_path = ctx.attr.name
    outputs = []
    registered_packages = []
    for plugin in plugins:
        plugin_target_name = plugin.target_name
        types_to_bases_and_names = plugin.types_to_bases_and_names

        base_package = _get_package_name(types_to_bases_and_names.values()[0][0])
        if base_package not in registered_packages:
            outputs.append(_write_package_xml(ctx, prefix_path, base_package))
            registered_packages.append(base_package)

        plugin_package = _get_package_name(types_to_bases_and_names.keys()[0])
        if plugin_package not in registered_packages:
            outputs.append(_write_package_xml(ctx, prefix_path, plugin_package))
            registered_packages.append(base_package)

        outputs.append(_write_plugin_manifest(
            ctx,
            prefix_path,
            base_package,
            plugin_package,
            plugin_target_name,
        ))
        outputs.append(_write_plugins_xml(
            ctx,
            prefix_path,
            plugin_package,
            plugin_target_name,
            types_to_bases_and_names,
        ))

        dynamic_library = ctx.actions.declare_file(
            paths.join(prefix_path, "lib", "lib" + plugin_target_name + ".so"),
        )
        ctx.actions.symlink(
            output = dynamic_library,
            target_file = plugin.dynamic_library,
        )
        outputs.append(dynamic_library)

    idl_plugins = depset(
        transitive = [
            dep[Ros2IdlPluginAspectInfo].plugins
            for dep in ctx.attr.idl_deps
        ],
    ).to_list()

    for plugin in idl_plugins:
        package_name = plugin.package_name

        if package_name not in registered_packages:
            outputs.append(_write_package_xml(ctx, prefix_path, package_name))
            registered_packages.append(package_name)
        dynamic_library = ctx.actions.declare_file(
            paths.join(prefix_path, "lib", "lib" + package_name + "__" + "rosidl_typesupport_cpp" + ".so"),
        )
        ctx.actions.symlink(
            output = dynamic_library,
            target_file = plugin.dynamic_library,
        )
        outputs.append(dynamic_library)

    idls = depset(
        transitive = [
            depset([
                struct(
                    package_name = dep.label.name,
                    srcs = dep[Ros2InterfaceInfo].info.srcs,
                ),
            ])
            for dep in ctx.attr.idl_deps
        ],
    ).to_list()

    for idl in idls:
        package_name = idl.package_name
        if package_name not in registered_packages:
            outputs.append(_write_package_xml(ctx, prefix_path, package_name))
            registered_packages.append(package_name)
        for src in idl.srcs:
            if src.extension != "msg":
                continue
            src_file = ctx.actions.declare_file(
                paths.join(prefix_path, "share", package_name, "msg", src.basename),
            )
            ctx.actions.symlink(
                output = src_file,
                target_file = src,
            )
            outputs.append(src_file)

    ament_prefix_path = None
    if outputs:
        ament_prefix_path = paths.join(ctx.attr.package_name, prefix_path)

    outputs_depset = depset(outputs)
    return [
        DefaultInfo(
            files = outputs_depset,
            runfiles = ctx.runfiles(transitive_files = outputs_depset),
        ),
        Ros2AmentSetupInfo(
            ament_prefix_path = ament_prefix_path,
        ),
    ]

ros2_ament_setup = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [ros2_plugin_collector_aspect],
        ),
        "idl_deps": attr.label_list(
            aspects = [
                idl_adapter_aspect,
                cpp_generator_aspect,
                ros2_idl_plugin_aspect,
            ],
            providers = [Ros2InterfaceInfo],
        ),
        "package_name": attr.string(
            mandatory = True,
        ),
    },
    implementation = _ros2_ament_setup_rule_impl,
)

def py_create_ament_setup(ament_prefix_path):
    """ The client code must do `import os`. """
    if ament_prefix_path == None:
        return "os.unsetenv('AMENT_PREFIX_PATH')"
    return "os.environ['AMENT_PREFIX_PATH'] = '{}'".format(ament_prefix_path)

def _py_launcher_rule_impl(ctx):
    output = ctx.actions.declare_file(ctx.attr.name + ".py")
    ament_prefix_path = ctx.attr.ament_setup[Ros2AmentSetupInfo].ament_prefix_path

    substitutions = dicts.add(
        ctx.attr.substitutions,
        {
            "{ament_setup}": py_create_ament_setup(ament_prefix_path),
        },
    )

    expand_template_impl(
        ctx,
        template = ctx.file.template,
        output = output,
        substitutions = substitutions,
        is_executable = False,
    )

    files = depset([output])
    runfiles = ctx.runfiles(transitive_files = files)
    runfiles = runfiles.merge(ctx.attr.ament_setup[DefaultInfo].default_runfiles)
    return [
        DefaultInfo(
            files = files,
            runfiles = runfiles,
        ),
        PyInfo(
            transitive_sources = files,
        ),
    ]

py_launcher_rule = rule(
    attrs = {
        "ament_setup": attr.label(
            mandatory = True,
            providers = [Ros2AmentSetupInfo],
        ),
        "template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "substitutions": attr.string_dict(mandatory = True),
        "data": attr.label_list(allow_files = True),
    },
    implementation = _py_launcher_rule_impl,
)

def py_launcher(name, deps, idl_deps = None, **kwargs):
    ament_setup = name + "_ament_setup"
    testonly = kwargs.get("testonly", False)
    ros2_ament_setup(
        name = ament_setup,
        deps = deps,
        idl_deps = idl_deps,
        package_name = native.package_name(),
        tags = ["manual"],
        testonly = testonly,
    )
    py_launcher_rule(
        name = name,
        ament_setup = ament_setup,
        **kwargs
    )
    return name + ".py"

SH_TOOLCHAIN = "@bazel_tools//tools/sh:toolchain_type"

def _sh_launcher_rule_impl(ctx):
    output = ctx.actions.declare_file(ctx.attr.name)
    ament_prefix_path = ctx.attr.ament_setup[Ros2AmentSetupInfo].ament_prefix_path

    substitutions = dicts.add(
        ctx.attr.substitutions,
        {
            "{{bash_bin}}": ctx.toolchains[SH_TOOLCHAIN].path,
            "{{ament_prefix_path}}": ament_prefix_path,
        },
    )

    expand_template_impl(
        ctx,
        template = ctx.file.template,
        output = output,
        substitutions = substitutions,
        is_executable = True,
    )

    files = depset([output])
    runfiles = ctx.runfiles(transitive_files = files)
    runfiles = runfiles.merge(ctx.attr.ament_setup[DefaultInfo].default_runfiles)
    return [
        DefaultInfo(
            files = files,
            runfiles = runfiles,
        ),
    ]

sh_launcher_rule = rule(
    attrs = {
        "ament_setup": attr.label(
            mandatory = True,
            providers = [Ros2AmentSetupInfo],
        ),
        "template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "substitutions": attr.string_dict(mandatory = True),
        "data": attr.label_list(allow_files = True),
    },
    implementation = _sh_launcher_rule_impl,
    toolchains = [SH_TOOLCHAIN],
)

def sh_launcher(name, deps, idl_deps = None, **kwargs):
    ament_setup = name + "_ament_setup"
    testonly = kwargs.get("testonly", False)
    ros2_ament_setup(
        name = ament_setup,
        deps = deps,
        idl_deps = idl_deps,
        package_name = native.package_name(),
        tags = ["manual"],
        testonly = testonly,
    )
    sh_launcher_rule(
        name = name,
        ament_setup = ament_setup,
        **kwargs
    )
