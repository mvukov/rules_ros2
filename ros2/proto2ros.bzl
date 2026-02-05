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
"""Rules for converting Protocol Buffers to ROS 2 messages."""

load("@protobuf//bazel/common:proto_info.bzl", "ProtoInfo")

def _proto2ros_message_impl(ctx):
    """Generates ROS 2 .msg files from proto_library descriptors."""
    proto_info = ctx.attr.proto_library[ProtoInfo]
    descriptor_set = proto_info.direct_descriptor_set
    proto_sources = proto_info.direct_sources
    msg_names = ctx.attr.msg_names
    label_name = ctx.label.name

    # Generate Python protobuf code for proto2ros to import.
    py_pb2_dir = ctx.actions.declare_directory(
        "{}_py_pb2".format(label_name),
    )

    proto_gen_args = ctx.actions.args()
    proto_gen_args.add("--python_out", py_pb2_dir.path)
    proto_gen_args.add_all(proto_sources)

    ctx.actions.run(
        inputs = proto_sources,
        outputs = [py_pb2_dir],
        executable = ctx.executable._protoc,
        arguments = [proto_gen_args],
        mnemonic = "GenProtoPython",
        progress_message = "Generating Python protobuf code for {}".format(label_name),
    )

    # Declare output files.
    output_prefix = "{}_proto2ros_output".format(label_name)

    msg_files = []
    for msg_name in msg_names:
        msg_file = ctx.actions.declare_file(
            "{}/msg/{}.msg".format(output_prefix, msg_name),
        )
        msg_files.append(msg_file)

    # Derive the output directory path from declared file paths.
    output_dir_path = msg_files[0].path.rsplit("/msg/", 1)[0]

    args = ctx.actions.args()
    args.add("--output-directory", output_dir_path)
    args.add(label_name)
    args.add(descriptor_set.path)

    ctx.actions.run(
        inputs = [descriptor_set, py_pb2_dir],
        outputs = msg_files,
        executable = ctx.executable._proto2ros_generate,
        arguments = [args],
        env = {"PYTHONPATH": py_pb2_dir.path},
        mnemonic = "Proto2RosGenerate",
        progress_message = "Generating ROS messages from proto for {}".format(label_name),
    )

    return [DefaultInfo(files = depset(msg_files))]

proto2ros_message = rule(
    implementation = _proto2ros_message_impl,
    attrs = {
        "proto_library": attr.label(
            mandatory = True,
            providers = [ProtoInfo],
            doc = "The proto_library target to convert to ROS messages.",
        ),
        "msg_names": attr.string_list(
            mandatory = True,
            doc = "List of message names that will be generated (e.g., ['ChatterMessage']).",
        ),
        "_proto2ros_generate": attr.label(
            default = Label("@proto2ros//:generate"),
            executable = True,
            cfg = "exec",
        ),
        "_protoc": attr.label(
            default = Label("@protobuf//src/google/protobuf/compiler:protoc"),
            executable = True,
            cfg = "exec",
        ),
    },
    doc = "Generates ROS 2 .msg files from a proto_library target.",
)
