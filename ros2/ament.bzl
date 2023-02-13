""" Defines ament-related utilities.
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "Ros2InterfaceInfo",
    "cpp_generator_aspect",
    "idl_adapter_aspect",
)
load(
    "@com_github_mvukov_rules_ros2//ros2:plugin.bzl",
    "Ros2IdlPluginAspectInfo",
    "Ros2PluginCollectorAspectInfo",
    "ros2_idl_plugin_aspect",
    "ros2_plugin_collector_aspect",
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
_PLUGINS_XML = "plugins.xml"

def _write_package_xml(ctx, prefix_path, package_name):
    package_xml = ctx.actions.declare_file(
        paths.join(prefix_path, _PACKAGES_PATH, package_name, _PACKAGE_XML),
    )
    ctx.actions.write(package_xml, _PACKAGE_XML_TEMPLATE.format(package_name = package_name))
    return package_xml

def _write_plugin_manifest(ctx, prefix_path, base_package, plugin_package):
    manifest = ctx.actions.declare_file(
        paths.join(
            prefix_path,
            _RESOURCE_INDEX_PATH,
            base_package + "__pluginlib__plugin",
            plugin_package + "_manifest",
        ),
    )
    ctx.actions.write(manifest, paths.join(_PACKAGES_PATH, plugin_package, _PLUGINS_XML))
    return manifest

_PLUGINS_XML_TEMPLATE = """\
<library path="{plugin_package}">
{plugins}
</library>
"""

_PLUGIN_XML_TEMPLATE = """\
<class name="{class_name}" type="{class_type}" base_class_type="{base_class_type}">
<description>{class_type}</description>
</class>
"""

def _write_plugins_xml(ctx, prefix_path, plugin_package, types_to_bases_and_names):
    plugins_xml = ctx.actions.declare_file(
        paths.join(prefix_path, _PACKAGES_PATH, plugin_package, _PLUGINS_XML),
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
        plugin_package = plugin_package,
        plugins = "\n".join(plugins),
    ))
    return plugins_xml

def _get_package_name(class_name):
    return class_name.split("::")[0]

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
        types_to_bases_and_names = plugin.types_to_bases_and_names

        base_package = _get_package_name(types_to_bases_and_names.values()[0][0])
        if base_package not in registered_packages:
            outputs.append(_write_package_xml(ctx, prefix_path, base_package))
            registered_packages.append(base_package)

        plugin_package = _get_package_name(types_to_bases_and_names.keys()[0])
        if plugin_package not in registered_packages:
            outputs.append(_write_package_xml(ctx, prefix_path, plugin_package))
            registered_packages.append(base_package)

        outputs.append(_write_plugin_manifest(ctx, prefix_path, base_package, plugin_package))
        outputs.append(_write_plugins_xml(
            ctx,
            prefix_path,
            plugin_package,
            types_to_bases_and_names,
        ))

        dynamic_library = ctx.actions.declare_file(
            paths.join(prefix_path, "lib", "lib" + plugin_package + ".so"),
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

    ament_setup = ctx.actions.declare_file(paths.join(prefix_path, _AMENT_SETUP_MODULE + ".py"))
    ament_prefix_path = "None"
    if outputs:
        ament_prefix_path = "'{}'".format(paths.join(ctx.attr.package_name, prefix_path))
    ctx.actions.expand_template(
        template = ctx.file._template,
        output = ament_setup,
        substitutions = {
            "{ament_prefix_path}": ament_prefix_path,
        },
    )
    outputs.append(ament_setup)

    outputs_depset = depset(outputs)
    return [
        DefaultInfo(
            files = outputs_depset,
            runfiles = ctx.runfiles(transitive_files = outputs_depset),
        ),
        PyInfo(
            transitive_sources = depset([ament_setup]),
        ),
    ]

ros2_ament_setup_rule = rule(
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
        "_template": attr.label(
            default = Label("@com_github_mvukov_rules_ros2//ros2:ament_setup.py.tpl"),
            allow_single_file = True,
        ),
    },
    implementation = _ros2_ament_setup_rule_impl,
)

def ros2_ament_setup(name, **kwargs):
    package_name = native.package_name()
    ros2_ament_setup_rule(
        name = name,
        package_name = package_name,
        **kwargs
    )

    py_module = "{}.{}".format(name, _AMENT_SETUP_MODULE)
    return py_module
