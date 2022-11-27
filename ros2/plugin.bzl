# Copyright 2022 Milan Vukov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
""" ROS2 plugin definitions.
"""

load("@rules_cc//cc:toolchain_utils.bzl", "find_cpp_toolchain")

Ros2PluginInfo = provider(
    "Provides necessary info for plugin routing",
    fields = [
        "dynamic_library",
        "base_class_type",
        "class_types",
    ],
)

def _ros2_plugin_impl(ctx):
    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    cc_info = ctx.attr.dep[CcInfo]
    linking_outputs = cc_common.link(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        linking_contexts = [cc_info.linking_context],
        name = ctx.attr.name + "__plugin__",
        output_type = "dynamic_library",
    )
    dynamic_library = linking_outputs.library_to_link.resolved_symlink_dynamic_library

    # TODO(mvukov) The given library might have data-deps: those must be propagated!

    return [
        DefaultInfo(
            files = depset([dynamic_library]),
        ),
        Ros2PluginInfo(
            dynamic_library = dynamic_library,
            base_class_type = ctx.attr.base_class_type,
            class_types = ctx.attr.class_types,
        ),
    ]

ros2_plugin = rule(
    attrs = {
        "dep": attr.label(
            providers = [CcInfo],
            mandatory = True,
        ),
        "base_class_type": attr.string(
            mandatory = True,
        ),
        "class_types": attr.string_list(
            mandatory = True,
        ),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    implementation = _ros2_plugin_impl,
    provides = [Ros2PluginInfo],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)

Ros2PluginCollectorAspectInfo = provider(
    "TBD",
    fields = [
        "plugins",
    ],
)

_ROS2_PLUGIN_COLLECTOR_ATTR_ASPECTS = ["data", "deps"]

def _get_list_attr(rule_attr, attr_name):
    if not hasattr(rule_attr, attr_name):
        return []
    candidate = getattr(rule_attr, attr_name)
    if type(candidate) != "list":
        fail("Expected a list for attribute `{}`!".format(attr_name))
    return candidate

def _collect_deps(rule_attr, attr_name, aspect_info):
    return [
        dep
        for dep in _get_list_attr(rule_attr, attr_name)
        if type(dep) == "Target" and aspect_info in dep
    ]

def _ros2_plugin_collector_aspect_impl(target, ctx):
    direct_plugins = []
    if ctx.rule.kind == "ros2_plugin":
        info = target[Ros2PluginInfo]
        plugin = struct(
            dynamic_library = info.dynamic_library,
            base_class_type = info.base_class_type,
            class_types = info.class_types,
        )
        direct_plugins.append(plugin)

    transitive_plugins = []
    for attr_name in _ROS2_PLUGIN_COLLECTOR_ATTR_ASPECTS:
        for dep in _collect_deps(ctx.rule.attr, attr_name, Ros2PluginCollectorAspectInfo):
            transitive_plugins.append(dep[Ros2PluginCollectorAspectInfo].plugins)

    return [
        Ros2PluginCollectorAspectInfo(
            plugins = depset(
                direct = direct_plugins,
                transitive = transitive_plugins,
            ),
        ),
    ]

ros2_plugin_collector_aspect = aspect(
    implementation = _ros2_plugin_collector_aspect_impl,
    attr_aspects = _ROS2_PLUGIN_COLLECTOR_ATTR_ASPECTS,
    provides = [Ros2PluginCollectorAspectInfo],
)

PACKAGE_XML_TEMPLATE = """\
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="2">
  <name>{package_name}</name>
</package>
"""

def write_package_xml(ctx, prefix_path, package_name):
    package_xml = ctx.actions.declare_file(prefix_path + "/share/ament_index/resource_index/packages/" + package_name + "/package.xml")
    ctx.actions.write(package_xml, PACKAGE_XML_TEMPLATE.format(package_name = package_name))
    return package_xml

def write_plugin_manifest(ctx, prefix_path, base_package, plugin_package):
    manifest = ctx.actions.declare_file(prefix_path +
                                        "/share/ament_index/resource_index/" + base_package + "__pluginlib__plugin/" + plugin_package + "_manifest")
    ctx.actions.write(manifest, "share/ament_index/resource_index/packages/" + plugin_package + "/plugins.xml")
    return manifest

PLUGINS_XML_TEMPLATE = """\
<library path="{plugin_package}">
{plugins}
</library>
"""

PLUGIN_XML_TEMPLATE = """\
<class type="{class_type}" base_class_type="{base_class_type}">
<description>{class_type}</description>
</class>
"""

def write_plugins_xml(ctx, prefix_path, plugin_package, base_class_type, class_types):
    plugins_xml = ctx.actions.declare_file(prefix_path + "/share/ament_index/resource_index/packages/" + plugin_package + "/plugins.xml")
    plugins = [
        PLUGIN_XML_TEMPLATE.format(
            class_type = class_type,
            base_class_type = base_class_type,
        )
        for class_type in class_types
    ]
    ctx.actions.write(plugins_xml, PLUGINS_XML_TEMPLATE.format(
        plugin_package = plugin_package,
        plugins = "\n".join(plugins),
    ))
    return plugins_xml

def _ros2_ament_setup_impl(ctx):
    plugins = depset(
        transitive = [
            dep[Ros2PluginCollectorAspectInfo].plugins
            for dep in ctx.attr.deps
        ],
    ).to_list()

    prefix_path = ctx.attr.name
    base_packages = []
    outputs = []
    for plugin in plugins:
        base_package = plugin.base_class_type.split("::")[0]
        if base_package not in base_packages:
            outputs.append(write_package_xml(ctx, prefix_path, base_package))

        plugin_package = plugin.class_types[0].split("::")[0]
        outputs.append(write_package_xml(ctx, prefix_path, plugin_package))

        outputs.append(write_plugin_manifest(ctx, prefix_path, base_package, plugin_package))
        outputs.append(write_plugins_xml(ctx, prefix_path, plugin_package, plugin.base_class_type, plugin.class_types))

        dynamic_library = ctx.actions.declare_file(
            prefix_path + "/lib/lib" + plugin_package + ".so",
        )
        ctx.actions.symlink(
            output = dynamic_library,
            target_file = plugin.dynamic_library,
        )
        outputs.append(dynamic_library)

    outputs_depset = depset(outputs)

    return [
        DefaultInfo(
            files = outputs_depset,
            runfiles = ctx.runfiles(transitive_files = outputs_depset),
        ),
    ]

ros2_ament_setup = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [ros2_plugin_collector_aspect],
        ),
    },
    implementation = _ros2_ament_setup_impl,
)
