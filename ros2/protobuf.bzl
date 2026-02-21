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
- Only proto3 scalar field types are supported (no message-type or enum fields).
- Repeated scalar fields map to dynamic ROS2 arrays (e.g. `int32[] values`).
- Proto `bytes` fields map to `uint8[]` in ROS2.

Example usage:
    proto_library(
        name = "my_proto",
        srcs = ["my.proto"],
    )

    proto_ros2_interface_library(
        name = "my_msgs",
        proto_deps = [":my_proto"],
    )

    cpp_ros2_interface_library(
        name = "cpp_my_msgs",
        deps = [":my_msgs"],
    )
"""

load("@com_github_mvukov_rules_ros2//ros2:interfaces.bzl", "Ros2InterfaceInfo")
load("@rules_proto//proto:defs.bzl", "ProtoInfo")

# Provider carrying the generated .msg files from a proto_library target.
ProtoToRos2MsgInfo = provider(
    "Provides generated .msg files derived from a proto_library target.",
    fields = {
        "msg_files": "A depset of generated .msg Files.",
    },
)

def _proto_to_ros2_msg_aspect_impl(target, ctx):
    proto_info = target[ProtoInfo]
    msg_files = []

    for src in proto_info.direct_sources:
        if not src.basename.endswith(".proto"):
            fail("Expected a .proto source file, got: {}".format(src.basename))
        stem = src.basename[:-len(".proto")]
        msg_file = ctx.actions.declare_file(
            "{}/{}.msg".format(target.label.name, stem),
        )
        msg_files.append(msg_file)

        ctx.actions.run(
            executable = ctx.executable._proto_to_ros2_msg,
            inputs = [proto_info.direct_descriptor_set],
            outputs = [msg_file],
            arguments = [
                "--descriptor_set",
                proto_info.direct_descriptor_set.path,
                "--proto_source",
                src.short_path,
                "--output",
                msg_file.path,
            ],
            mnemonic = "ProtoToRos2Msg",
            progress_message = "Converting proto to ROS2 msg for %{label}",
        )

    return [ProtoToRos2MsgInfo(msg_files = depset(msg_files))]

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
    provides = [ProtoToRos2MsgInfo],
)

def _proto_ros2_interface_library_impl(ctx):
    msg_files = []
    for dep in ctx.attr.proto_deps:
        msg_files.extend(dep[ProtoToRos2MsgInfo].msg_files.to_list())

    return [
        DefaultInfo(files = depset(msg_files)),
        Ros2InterfaceInfo(
            info = struct(srcs = msg_files),
            deps = depset(
                direct = [dep[Ros2InterfaceInfo].info for dep in ctx.attr.deps],
                transitive = [
                    dep[Ros2InterfaceInfo].deps
                    for dep in ctx.attr.deps
                ],
            ),
        ),
    ]

proto_ros2_interface_library = rule(
    implementation = _proto_ros2_interface_library_impl,
    attrs = {
        # Named `proto_deps` rather than `deps` so that aspects propagating
        # along `deps` (e.g. idl_adapter_aspect) do not follow this edge into
        # proto_library targets, which do not carry Ros2InterfaceInfo.
        "proto_deps": attr.label_list(
            providers = [ProtoInfo],
            aspects = [proto_to_ros2_msg_aspect],
            mandatory = True,
            doc = "List of proto_library targets to convert to ROS2 interfaces.",
        ),
        # Standard interface deps (other ros2_interface_library /
        # proto_ros2_interface_library targets). Enables dependency on other
        # message packages and satisfies ctx.rule.attr.deps accesses in the
        # language-generator aspects (idl_adapter_aspect, c_generator_aspect,
        # etc.).
        "deps": attr.label_list(
            providers = [Ros2InterfaceInfo],
            doc = "ROS2 interface libraries this target depends on.",
        ),
    },
    provides = [Ros2InterfaceInfo],
    doc = """Converts proto_library targets to a ROS2 interface library.

Generates one .msg file per proto source file and exposes Ros2InterfaceInfo
so that downstream rules such as cpp_ros2_interface_library and
py_ros2_interface_library can consume the result.""",
)
