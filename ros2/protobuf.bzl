# Copyright 2026 Milan Vukov
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
"""Converts proto_library targets to ros2_interface_library-compatible targets.

Limitations:
- One proto file must correspond to exactly one message definition.
- Service definitions in a proto file cause a build error.
- Message-type fields are supported as cross-package ROS2 references (e.g.
  `pkg/Type`). Each proto dep must have a corresponding
  proto_ros2_interface_library so the package name can be resolved.
- Enum and group fields are not supported.
- Repeated scalar and message fields map to dynamic ROS2 arrays.
- Proto `bytes` fields map to `uint8[]` in ROS2.

Example usage:
    proto_library(
        name = "my_proto",
        srcs = ["my.proto"],
    )

    proto_ros2_interface_library(
        name = "my_msgs",
        deps = [":my_proto"],
    )

    cpp_ros2_interface_library(
        name = "cpp_my_msgs",
        deps = [":my_msgs"],
    )
"""

load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "CppGeneratorAspectInfo",
    "Ros2InterfaceInfo",
    "cc_generator_impl",
    "cpp_generator_aspect",
    "idl_adapter_aspect",
)
load("@com_google_protobuf//bazel/common:proto_info.bzl", "ProtoInfo")
load("@com_google_protobuf//bazel/private:cc_proto_aspect.bzl", "cc_proto_aspect")
load("@rules_cc//cc:toolchain_utils.bzl", "find_cpp_toolchain")

CppProtoConverterAspectInfo = provider("TBD", fields = ["cc_info"])

def _proto_to_ros2_msg_aspect_impl(target, ctx):
    proto_info = target[ProtoInfo]
    msg_files = []

    ros_package_name = target.label.name + "_ros_msgs"

    dep_extra_args = []
    dep_descriptor_sets = []
    for dep in ctx.rule.attr.deps:
        dep_ds = dep[ProtoInfo].direct_descriptor_set
        dep_descriptor_sets.append(dep_ds)
        dep_extra_args += ["--dep_descriptor_set", dep_ds.path]
        dep_ros2_package = dep[Ros2InterfaceInfo].ros_package_name
        for src in dep[ProtoInfo].direct_sources:
            dep_extra_args += [
                "--dep_mapping",
                "{}:{}".format(src.short_path, dep_ros2_package),
            ]

    for src in proto_info.direct_sources:
        if not src.basename.endswith(".proto"):
            fail("Expected a .proto source file, got: {}".format(src.basename))
        stem = src.basename[:-len(".proto")].capitalize()
        msg_file = ctx.actions.declare_file(
            "{}/{}.msg".format(ros_package_name, stem),
        )
        msg_files.append(msg_file)

        ctx.actions.run(
            executable = ctx.executable._proto_to_ros2_msg,
            inputs = [proto_info.direct_descriptor_set] + dep_descriptor_sets,
            outputs = [msg_file],
            arguments = [
                "--descriptor_set",
                proto_info.direct_descriptor_set.path,
                "--proto_source",
                src.short_path,
                "--output",
                msg_file.path,
            ] + dep_extra_args,
            mnemonic = "ProtoToRos2Msg",
            progress_message = "Converting proto to ROS 2 messages for %{label}",
        )

    return [
        Ros2InterfaceInfo(
            info = struct(srcs = msg_files),
            deps = depset(
                direct = [dep[Ros2InterfaceInfo].info for dep in ctx.rule.attr.deps],
                transitive = [
                    dep[Ros2InterfaceInfo].deps
                    for dep in ctx.rule.attr.deps
                ],
            ),
            ros_package_name = ros_package_name,
        ),
    ]

proto_to_ros2_msg_aspect = aspect(
    implementation = _proto_to_ros2_msg_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_proto_to_ros2_msg": attr.label(
            default = Label("@com_github_mvukov_rules_ros2//ros2:proto_to_ros2_msg"),
            executable = True,
            cfg = "exec",
        ),
    },
    required_providers = [ProtoInfo],
    provides = [Ros2InterfaceInfo],
)

def _cpp_proto_ros2_interface_library_impl(ctx):
    return cc_generator_impl(ctx, CppGeneratorAspectInfo)

cpp_proto_ros2_interface_library = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [proto_to_ros2_msg_aspect, idl_adapter_aspect, cpp_generator_aspect],
            providers = [ProtoInfo],
        ),
    },
    implementation = _cpp_proto_ros2_interface_library_impl,
)

def _cpp_proto_ros2_converter_aspect_impl(target, ctx):
    proto_info = target[ProtoInfo]
    ros_package_name = target[Ros2InterfaceInfo].ros_package_name

    # Collect dep information: descriptor sets and proto→ros_package mappings.
    dep_extra_args = []
    dep_descriptor_sets = []
    dep_converter_cc_infos = []
    for dep in ctx.rule.attr.deps:
        dep_ds = dep[ProtoInfo].direct_descriptor_set
        dep_descriptor_sets.append(dep_ds)
        dep_extra_args += ["--dep_descriptor_set", dep_ds.path]
        dep_ros2_package = dep[Ros2InterfaceInfo].ros_package_name
        for src in dep[ProtoInfo].direct_sources:
            dep_extra_args += [
                "--dep_mapping",
                "{}:{}".format(src.short_path, dep_ros2_package),
            ]
        if CppProtoConverterAspectInfo in dep:
            dep_converter_cc_infos.append(dep[CppProtoConverterAspectInfo].cc_info)

    # Declare output files.
    header = ctx.actions.declare_file(
        "{}/proto_converters.h".format(ros_package_name),
    )
    source = ctx.actions.declare_file(
        "{}/proto_converters.cc".format(ros_package_name),
    )

    ctx.actions.run(
        executable = ctx.executable._proto_to_ros2_converter,
        inputs = [proto_info.direct_descriptor_set] + dep_descriptor_sets,
        outputs = [header, source],
        arguments = [
            "--descriptor_set",
            proto_info.direct_descriptor_set.path,
            "--ros_package_name",
            ros_package_name,
            "--output_header",
            header.path,
            "--output_source",
            source.path,
        ] + dep_extra_args,
        mnemonic = "ProtoToRos2Converter",
        progress_message = "Generating proto/ROS2 converters for %{label}",
    )

    # The include root is the parent directory of the ros_package_name folder.
    cc_include_dir = "/".join(header.dirname.split("/")[:-1])

    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    compilation_contexts = (
        [
            target[CcInfo].compilation_context,
            target[CppGeneratorAspectInfo].cc_info.compilation_context,
        ] +
        [ci.compilation_context for ci in dep_converter_cc_infos]
    )
    compilation_context, compilation_outputs = cc_common.compile(
        actions = ctx.actions,
        name = ros_package_name + "_converter",
        cc_toolchain = cc_toolchain,
        feature_configuration = feature_configuration,
        system_includes = [cc_include_dir],
        srcs = [source],
        public_hdrs = [header],
        compilation_contexts = compilation_contexts,
    )

    linking_contexts = (
        [
            target[CcInfo].linking_context,
            target[CppGeneratorAspectInfo].cc_info.linking_context,
        ] +
        [ci.linking_context for ci in dep_converter_cc_infos]
    )
    linking_context, _ = cc_common.create_linking_context_from_compilation_outputs(
        actions = ctx.actions,
        name = ros_package_name + "_converter",
        compilation_outputs = compilation_outputs,
        cc_toolchain = cc_toolchain,
        feature_configuration = feature_configuration,
        linking_contexts = linking_contexts,
    )

    return [
        CppProtoConverterAspectInfo(
            cc_info = CcInfo(
                compilation_context = compilation_context,
                linking_context = linking_context,
            ),
        ),
    ]

cpp_proto_ros2_converter_aspect = aspect(
    implementation = _cpp_proto_ros2_converter_aspect_impl,
    attr_aspects = ["deps"],
    attrs = {
        "_proto_to_ros2_converter": attr.label(
            default = Label("@com_github_mvukov_rules_ros2//ros2:proto_to_ros2_converter"),
            executable = True,
            cfg = "exec",
        ),
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    required_providers = [ProtoInfo],
    required_aspect_providers = [
        [Ros2InterfaceInfo],
        [CppGeneratorAspectInfo],
        [CcInfo],
    ],
    provides = [CppProtoConverterAspectInfo],
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)

def _cpp_proto_ros2_converter_library_impl(ctx):
    cc_info = cc_common.merge_cc_infos(
        direct_cc_infos = [
            dep[CppProtoConverterAspectInfo].cc_info
            for dep in ctx.attr.deps
        ],
    )
    return [cc_info]

cpp_proto_ros2_converter_library = rule(
    attrs = {
        "deps": attr.label_list(
            mandatory = True,
            aspects = [
                proto_to_ros2_msg_aspect,
                idl_adapter_aspect,
                cpp_generator_aspect,
                cc_proto_aspect,
                cpp_proto_ros2_converter_aspect,
            ],
            providers = [ProtoInfo],
        ),
    },
    implementation = _cpp_proto_ros2_converter_library_impl,
)
