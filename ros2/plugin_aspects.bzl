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
""" ROS 2 plugin providers and aspects.
"""

load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "CppGeneratorAspectInfo",
    "IdlAdapterAspectInfo",
    "Ros2InterfaceInfo",
)
load("@rules_cc//cc:toolchain_utils.bzl", "find_cpp_toolchain")

Ros2PluginInfo = provider(
    "Provides necessary info for plugin routing.",
    fields = [
        "target_name",
        "dynamic_library",
        "types_to_bases_and_names",
    ],
)

Ros2PluginCollectorAspectInfo = provider(
    "Provides info about collected plugins.",
    fields = [
        "plugins",
    ],
)

_ROS2_COLLECTOR_ATTR_ASPECTS = ["data", "deps"]

def _get_list_attr(rule_attr, attr_name):
    if not hasattr(rule_attr, attr_name):
        return []
    candidate = getattr(rule_attr, attr_name)
    if type(candidate) != "list":
        fail("Expected a list for attribute `{}`!".format(attr_name))
    return candidate

def _collect_deps(rule_attr, attr_name, provider_info):
    return [
        dep
        for dep in _get_list_attr(rule_attr, attr_name)
        if type(dep) == "Target" and provider_info in dep
    ]

def _get_transitive_items(ctx, aspect_name, item_name):
    transitive_items = []
    for attr_name in _ROS2_COLLECTOR_ATTR_ASPECTS:
        for dep in _collect_deps(ctx.rule.attr, attr_name, aspect_name):
            transitive_items.append(getattr(dep[aspect_name], item_name))
    return transitive_items

def _ros2_plugin_collector_aspect_impl(target, ctx):
    direct_plugins = []
    if ctx.rule.kind == "ros2_plugin_rule":
        direct_plugins.append(target[Ros2PluginInfo])

    transitive_plugins = _get_transitive_items(
        ctx,
        Ros2PluginCollectorAspectInfo,
        "plugins",
    )

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
    attr_aspects = _ROS2_COLLECTOR_ATTR_ASPECTS,
    provides = [Ros2PluginCollectorAspectInfo],
)

Ros2InterfaceCollectorAspectInfo = provider(
    "Provides info about collected interfaces.",
    fields = [
        "interfaces",
    ],
)

def create_interface_struct(target):
    return struct(
        package_name = target.label.name,
        srcs = target[Ros2InterfaceInfo].info.srcs,
    )

def _ros2_interface_collector_aspect_impl(target, ctx):
    direct_interfaces = []
    if ctx.rule.kind == "ros2_interface_library":
        direct_interfaces.append(create_interface_struct(target))

    transitive_interfaces = _get_transitive_items(
        ctx,
        Ros2InterfaceCollectorAspectInfo,
        "interfaces",
    )

    return [
        Ros2InterfaceCollectorAspectInfo(
            interfaces = depset(
                direct = direct_interfaces,
                transitive = transitive_interfaces,
            ),
        ),
    ]

ros2_interface_collector_aspect = aspect(
    implementation = _ros2_interface_collector_aspect_impl,
    attr_aspects = _ROS2_COLLECTOR_ATTR_ASPECTS,
    provides = [Ros2InterfaceCollectorAspectInfo],
)

Ros2IdlPluginAspectInfo = provider(
    "Provides info for generated IDL plugins.",
    fields = [
        "plugins",
    ],
)

def create_dynamic_library(ctx, **kwargs):
    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    linking_outputs = cc_common.link(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        output_type = "dynamic_library",
        **kwargs
    )
    dynamic_library = linking_outputs.library_to_link.resolved_symlink_dynamic_library
    return dynamic_library

def _ros2_idl_plugin_aspect_impl(target, ctx):
    package_name = target.label.name
    cc_info = target[CppGeneratorAspectInfo].cc_info
    dynamic_library = create_dynamic_library(
        ctx,
        name = package_name + "/plugin",
        compilation_outputs = target[CppGeneratorAspectInfo].compilation_outputs,
        linking_contexts = [cc_info.linking_context],
    )
    plugin = struct(
        package_name = package_name,
        dynamic_library = dynamic_library,
    )

    return [
        Ros2IdlPluginAspectInfo(
            plugins = depset(
                direct = [plugin],
                transitive = [
                    dep[Ros2IdlPluginAspectInfo].plugins
                    for dep in ctx.rule.attr.deps
                ],
            ),
        ),
    ]

ros2_idl_plugin_aspect = aspect(
    implementation = _ros2_idl_plugin_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    required_providers = [Ros2InterfaceInfo],
    required_aspect_providers = [
        [IdlAdapterAspectInfo],
        [CppGeneratorAspectInfo],
    ],
    provides = [Ros2IdlPluginAspectInfo],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)
