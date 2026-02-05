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
""" ROS 2 plugin definitions.
"""

load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_library")
load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "CppGeneratorAspectInfo",
    "IdlAdapterAspectInfo",
    "Ros2InterfaceInfo",
    "cpp_generator_aspect",
    "idl_adapter_aspect",
)
load(
    "@com_github_mvukov_rules_ros2//ros2:plugin_aspects.bzl",
    "Ros2PluginInfo",
    "create_dynamic_library",
)
load("@rules_cc//cc:toolchain_utils.bzl", "find_cpp_toolchain")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

def _ros2_plugin_impl(ctx):
    target_name = ctx.attr.name
    name = target_name + "/plugin"
    dynamic_library = create_dynamic_library(
        ctx,
        name = name,
        linking_contexts = [ctx.attr.dep[CcInfo].linking_context],
    )

    return [
        DefaultInfo(
            files = depset([dynamic_library]),
            runfiles = ctx.attr.dep[DefaultInfo].default_runfiles,
        ),
        Ros2PluginInfo(
            target_name = target_name,
            dynamic_library = dynamic_library,
            types_to_bases_and_names = ctx.attr.types_to_bases_and_names,
        ),
    ]

ros2_plugin_rule = rule(
    attrs = {
        "dep": attr.label(
            providers = [CcInfo],
            mandatory = True,
        ),
        "types_to_bases_and_names": attr.string_list_dict(
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

def ros2_plugin(name, plugin_specs, **kwargs):
    types_to_bases_and_names = {}
    for plugin_spec in plugin_specs:
        class_type = plugin_spec["class_type"]
        class_name = plugin_spec.get("class_name", class_type)
        base_class_type = plugin_spec["base_class_type"]
        types_to_bases_and_names[class_type] = [base_class_type, class_name]

    lib_name = name + "_impl"
    tags = kwargs.pop("tags", None)
    visibility = kwargs.pop("visibility", None)
    ros2_cpp_library(
        name = lib_name,
        # This must be set such that static plugin registration works.
        alwayslink = True,
        tags = ["manual"],
        **kwargs
    )
    ros2_plugin_rule(
        name = name,
        types_to_bases_and_names = types_to_bases_and_names,
        dep = lib_name,
        tags = tags,
        visibility = visibility,
    )
