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
    "Ros2InterfaceCollectorAspectInfo",
    "Ros2PluginCollectorAspectInfo",
    "create_interface_struct",
    "ros2_idl_plugin_aspect",
    "ros2_interface_collector_aspect",
    "ros2_plugin_collector_aspect",
)
load(
    "@com_github_mvukov_rules_ros2//third_party:expand_template.bzl",
    "expand_template_impl",
)
load("@rules_cc//cc:defs.bzl", "cc_library")

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

    idls_from_deps = depset(
        transitive = [
            dep[Ros2InterfaceCollectorAspectInfo].interfaces
            for dep in ctx.attr.deps
        ],
    )
    idls = depset(
        transitive = [
            depset([create_interface_struct(dep)])
            for dep in ctx.attr.idl_deps
        ] + [idls_from_deps],
    ).to_list()

    for idl in idls:
        package_name = idl.package_name
        if package_name not in registered_packages:
            outputs.append(_write_package_xml(ctx, prefix_path, package_name))
            registered_packages.append(package_name)
        idl_manifest_contents = []
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
            idl_manifest_contents.append(paths.join("msg", src.basename))

        idl_manifest = ctx.actions.declare_file(
            paths.join(prefix_path, _RESOURCE_INDEX_PATH, "rosidl_interfaces", package_name),
        )
        ctx.actions.write(idl_manifest, "\n".join([p for p in idl_manifest_contents]))
        outputs.append(idl_manifest)

    ament_prefix_path = None
    if outputs:
        manifest = ctx.actions.declare_file(paths.join(prefix_path, "manifest.txt"))
        ctx.actions.write(manifest, "\n".join([p.short_path for p in outputs]))
        ament_prefix_path = paths.dirname(manifest.short_path)
        outputs.append(manifest)

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
            aspects = [
                ros2_interface_collector_aspect,
                ros2_plugin_collector_aspect,
            ],
        ),
        "idl_deps": attr.label_list(
            aspects = [
                idl_adapter_aspect,
                cpp_generator_aspect,
                ros2_idl_plugin_aspect,
            ],
            providers = [Ros2InterfaceInfo],
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
        "data": attr.label_list(allow_files = True),
        "substitutions": attr.string_dict(mandatory = True),
        "template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
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
        tags = ["manual"],
        testonly = testonly,
    )
    py_launcher_rule(
        name = name,
        ament_setup = ament_setup,
        **kwargs
    )
    return name + ".py"

def split_kwargs(**kwargs):
    """Split kwargs into those to be forwarded to the actual binary target and launcher target respectively."""
    launcher_attrs = ["args", "env", "flaky", "size", "tags", "timeout", "visibility"]
    launcher_target_kwargs = {attr: kwargs.pop(attr) for attr in launcher_attrs if attr in kwargs}
    return launcher_target_kwargs, kwargs

SH_TOOLCHAIN = "@bazel_tools//tools/sh:toolchain_type"

def _sh_launcher_rule_impl(ctx):
    output = ctx.actions.declare_file(ctx.attr.name)
    ament_prefix_path = ctx.attr.ament_setup[Ros2AmentSetupInfo].ament_prefix_path or ""

    substitutions = dicts.add(
        ctx.attr.substitutions,
        {
            "{{ament_prefix_path}}": ament_prefix_path,
            "{{bash_bin}}": ctx.toolchains[SH_TOOLCHAIN].path,
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
        "data": attr.label_list(allow_files = True),
        "substitutions": attr.string_dict(mandatory = True),
        "template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
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
        tags = ["manual"],
        testonly = testonly,
    )
    sh_launcher_rule(
        name = name,
        ament_setup = ament_setup,
        **kwargs
    )

def _cpp_ament_setup_impl(ctx):
    basename = ctx.attr.basename or ctx.label.name
    src_output = ctx.actions.declare_file(basename + "/ament_setup.cc")
    hdr_output = ctx.actions.declare_file(basename + "/ament_setup.h")
    ament_prefix_path = ctx.attr.ament_setup[Ros2AmentSetupInfo].ament_prefix_path

    substitutions = {
        "{{ament_prefix_path}}": ament_prefix_path,
        "{{header}}": hdr_output.short_path,
        "{{namespace}}": basename,
    }

    expand_template_impl(
        ctx,
        template = ctx.file._src_template,
        output = src_output,
        substitutions = substitutions,
        is_executable = False,
    )
    expand_template_impl(
        ctx,
        template = ctx.file._hdr_template,
        output = hdr_output,
        substitutions = substitutions,
        is_executable = False,
    )

    files = depset([src_output, hdr_output])
    return [
        DefaultInfo(
            files = files,
        ),
    ]

cpp_ament_setup = rule(
    attrs = {
        "ament_setup": attr.label(
            mandatory = True,
            providers = [Ros2AmentSetupInfo],
        ),
        "basename": attr.string(
            default = "",
        ),
        "data": attr.label_list(allow_files = True),
        "_hdr_template": attr.label(
            allow_single_file = True,
            default = "@com_github_mvukov_rules_ros2//ros2:ament_setup.h.tpl",
        ),
        "_src_template": attr.label(
            allow_single_file = True,
            default = "@com_github_mvukov_rules_ros2//ros2:ament_setup.cc.tpl",
        ),
    },
    implementation = _cpp_ament_setup_impl,
)

def cpp_ament_setup_library(name, deps, idl_deps = None, **kwargs):
    """ Generates a C++ library that contains the ament setup code for a ROS2 package.

    This can be useful in contexts where using a wrapper script (as is the case with the regular ros2_xxx rules)
    is not desirable. The generated library can be included as `#include <package_name>/<name>/ament_setup.h` and
    contains a single function called `SetUpAmentPrefixPath` in the namespace `<name>`. That function needs to be
    called before any ament resource is used.

    The generated function can be called with an optional boolean allow_append argument, which causes the
    function to append to the AMENT_PREFIX_PATH instead of overwriting it. Beware that when using this mode when
    a different version of the same plugin may exist in the existing AMENT_PREFIX_PATH, the behavior might be
    unexpected.

    Args:
        name: The name of the target.
        deps: The dependencies, typically targets of type ros2_plugin.
        idl_deps: Additional IDL deps that are used as runtime plugins.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes
    """
    ament_setup = name + "_ament_setup"
    testonly = kwargs.get("testonly", False)
    ros2_ament_setup(
        name = ament_setup,
        deps = deps,
        idl_deps = idl_deps,
        tags = ["manual"],
        testonly = testonly,
    )

    cpp_ament_setup(
        name = name + "_srcs",
        basename = name,
        ament_setup = ament_setup,
        tags = ["manual"],
    )

    cc_library(
        name = name,
        srcs = [":{}_srcs".format(name)],
        data = [ament_setup],
        testonly = testonly,
        **kwargs
    )
