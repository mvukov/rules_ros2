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
        deps = [":my_proto"],
    )

    cpp_ros2_interface_library(
        name = "cpp_my_msgs",
        deps = [":my_msgs"],
    )
"""

load("@com_github_mvukov_rules_ros2//ros2:interfaces.bzl", "Ros2InterfaceInfo")
load("@rules_proto//proto:defs.bzl", "ProtoInfo")

def _proto_to_ros2_msg_aspect_impl(target, ctx):
    proto_info = target[ProtoInfo]
    msg_files = []

    for src in proto_info.direct_sources:
        if not src.basename.endswith(".proto"):
            fail("Expected a .proto source file, got: {}".format(src.basename))
        stem = src.basename[:-len(".proto")].capitalize()
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

    return [
        Ros2InterfaceInfo(
            info = struct(srcs = msg_files),
            deps = depset([]),
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

def _proto_ros2_interface_library_impl(ctx):
    msg_files = []
    for dep in ctx.attr.deps:
        msg_files.extend(dep[Ros2InterfaceInfo].info.srcs)

    return [
        DefaultInfo(files = depset(msg_files)),
        Ros2InterfaceInfo(
            info = struct(srcs = msg_files),
            deps = depset([]),
        ),
    ]

proto_ros2_interface_library = rule(
    implementation = _proto_ros2_interface_library_impl,
    attrs = {
        "deps": attr.label_list(
            providers = [ProtoInfo],
            aspects = [proto_to_ros2_msg_aspect],
            mandatory = True,
            doc = "List of proto_library targets to convert to ROS2 interfaces.",
        ),
    },
    provides = [Ros2InterfaceInfo],
    doc = """Converts proto_library targets to a ROS2 interface library.

Generates one .msg file per proto source file and exposes Ros2InterfaceInfo
so that downstream rules such as cpp_ros2_interface_library and
py_ros2_interface_library can consume the result.""",
)
