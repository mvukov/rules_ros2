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
"""Converts a proto file to a ROS2 .msg file.

Limitations:
- Exactly one message definition per proto file is required.
- Service definitions are not supported.
- Message-type fields are supported as cross-package ROS2 references (e.g.
  `pkg/Type`). The caller must supply --dep_mapping for each imported proto.
- Enum and group fields are not supported.
- Repeated scalar and message fields are supported and map to dynamic arrays
  (e.g. `int32[] values`, `pkg/msg/Point[] points`).
- proto `bytes` fields map to `uint8[]` in ROS2.
"""
import argparse
import os
import sys

from google.protobuf import descriptor_pb2
from google.protobuf.descriptor_pb2 import FieldDescriptorProto

# Mapping from proto3 scalar FieldDescriptorProto.Type to ROS2 .msg type.
# Types that map to arrays (like bytes) use a special sentinel handled below.
_PROTO_TO_ROS2_TYPE = {
    FieldDescriptorProto.TYPE_DOUBLE:
        'float64',
    FieldDescriptorProto.TYPE_FLOAT:
        'float32',
    FieldDescriptorProto.TYPE_INT32:
        'int32',
    FieldDescriptorProto.TYPE_INT64:
        'int64',
    FieldDescriptorProto.TYPE_UINT32:
        'uint32',
    FieldDescriptorProto.TYPE_UINT64:
        'uint64',
    FieldDescriptorProto.TYPE_SINT32:
        'int32',
    FieldDescriptorProto.TYPE_SINT64:
        'int64',
    FieldDescriptorProto.TYPE_FIXED32:
        'uint32',
    FieldDescriptorProto.TYPE_FIXED64:
        'uint64',
    FieldDescriptorProto.TYPE_SFIXED32:
        'int32',
    FieldDescriptorProto.TYPE_SFIXED64:
        'int64',
    FieldDescriptorProto.TYPE_BOOL:
        'bool',
    FieldDescriptorProto.TYPE_STRING:
        'string',
    # bytes in proto3 → dynamic byte array in ROS2
    FieldDescriptorProto.TYPE_BYTES:
        'uint8[]',
}

# Non-scalar types that are explicitly rejected.
_UNSUPPORTED_TYPES = {
    FieldDescriptorProto.TYPE_GROUP: 'group',
    FieldDescriptorProto.TYPE_ENUM: 'enum',
}


def _build_msg_type_map(dep_descriptor_set_paths, dep_mapping,
                        main_descriptor_set_path, proto_source,
                        self_ros_package):
    """Build {'.pkg.MsgName': 'ros2_package/MsgName'} from dep descriptor sets.

    Also scans the main descriptor set for sibling files (other sources in the
    same proto_library target) and maps their message types to self_ros_package.
    """
    path_to_pkg = {}
    for entry in dep_mapping:
        proto_path, ros2_pkg = entry.split(':', 1)
        path_to_pkg[proto_path] = ros2_pkg

    msg_type_map = {}

    # Scan sibling files in the main descriptor set (same proto_library target).
    with open(main_descriptor_set_path, 'rb') as f:
        main_data = f.read()
    main_set = descriptor_pb2.FileDescriptorSet()
    main_set.ParseFromString(main_data)
    for file_proto in main_set.file:
        if file_proto.name == proto_source:
            continue  # Skip the file being converted.
        if file_proto.name in path_to_pkg:
            continue  # Already covered by a dep_mapping.
        # This is a sibling file; it belongs to the same ROS2 package.
        pkg_prefix = '.' + file_proto.package if file_proto.package else ''
        for msg in file_proto.message_type:
            fq = f'{pkg_prefix}.{msg.name}'
            msg_type_map[fq] = f'{self_ros_package}/{msg.name}'

    for ds_path in dep_descriptor_set_paths:
        with open(ds_path, 'rb') as f:
            data = f.read()
        dep_set = descriptor_pb2.FileDescriptorSet()
        dep_set.ParseFromString(data)
        for file_proto in dep_set.file:
            ros2_pkg = path_to_pkg.get(file_proto.name)
            if ros2_pkg is None:
                continue
            pkg_prefix = '.' + file_proto.package if file_proto.package else ''
            for msg in file_proto.message_type:
                fq = f'{pkg_prefix}.{msg.name}'
                msg_type_map[fq] = f'{ros2_pkg}/{msg.name}'
    return msg_type_map


def _find_file_descriptor(proto_set, proto_source):
    """Find a FileDescriptorProto by name, with fallback to basename matching.
    """
    for file_proto in proto_set.file:
        if file_proto.name == proto_source:
            return file_proto
    # Fallback: match by basename in case paths differ slightly.
    source_basename = proto_source.split('/')[-1]
    for file_proto in proto_set.file:
        if file_proto.name.split('/')[-1] == source_basename:
            return file_proto
    return None


def _convert(file_proto, output_path, proto_source, msg_type_map):
    """Validate and convert a FileDescriptorProto to a ROS2 .msg file."""
    if file_proto.service:
        sys.exit(f'Error: {proto_source}: services are not supported '
                 f'(found {len(file_proto.service)} service(s)).')

    num_messages = len(file_proto.message_type)
    if num_messages != 1:
        sys.exit(
            f'Error: {proto_source}: expected exactly 1 message definition, '
            f'got {num_messages}.')

    message = file_proto.message_type[0]
    lines = [f'# Generated from proto source: {proto_source}', '']

    for field in message.field:
        field_type_value = field.type
        is_repeated = (field.label == FieldDescriptorProto.LABEL_REPEATED)

        if field_type_value == FieldDescriptorProto.TYPE_MESSAGE:
            ros2_type = msg_type_map.get(field.type_name)
            if ros2_type is None:
                sys.exit(
                    f'Error: {proto_source}: field "{field.name}" references '
                    f'message type "{field.type_name}" with no dep_mapping '
                    f'entry. Add a --dep_mapping for the proto file that '
                    f'defines it.')
            if is_repeated:
                ros2_type = ros2_type + '[]'
            lines.append(f'{ros2_type} {field.name}')
            continue

        if field_type_value in _UNSUPPORTED_TYPES:
            type_name = _UNSUPPORTED_TYPES[field_type_value]
            sys.exit(
                f'Error: {proto_source}: field "{field.name}" has unsupported '
                f'type "{type_name}".')

        if field_type_value not in _PROTO_TO_ROS2_TYPE:
            sys.exit(f'Error: {proto_source}: field "{field.name}" has unknown '
                     f'type value {field_type_value}.')

        ros2_type = _PROTO_TO_ROS2_TYPE[field_type_value]

        # proto `bytes` already becomes `uint8[]`; avoid double `[]`.
        if is_repeated and field_type_value != FieldDescriptorProto.TYPE_BYTES:
            ros2_type = ros2_type + '[]'

        lines.append(f'{ros2_type} {field.name}')

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(
        description='Convert a proto file to a ROS2 .msg file.')
    parser.add_argument(
        '--descriptor_set',
        required=True,
        help='Path to the binary FileDescriptorSet file.',
    )
    parser.add_argument(
        '--proto_source',
        required=True,
        help='Relative path of the proto source as stored in the descriptor.',
    )
    parser.add_argument(
        '--output',
        required=True,
        help='Path of the output .msg file to write.',
    )
    parser.add_argument(
        '--dep_mapping',
        action='append',
        default=[],
        metavar='PROTO_PATH:ROS2_PACKAGE',
        help='Mapping from a dep proto file path to its ROS2 package name. '
        'May be repeated.',
    )
    parser.add_argument(
        '--dep_descriptor_set',
        action='append',
        default=[],
        metavar='PATH',
        help='Path to a dep binary FileDescriptorSet file. May be repeated.',
    )
    args = parser.parse_args()

    with open(args.descriptor_set, 'rb') as f:
        data = f.read()

    proto_set = descriptor_pb2.FileDescriptorSet()
    proto_set.ParseFromString(data)

    file_proto = _find_file_descriptor(proto_set, args.proto_source)
    if file_proto is None:
        sys.exit(f'Error: could not find proto source "{args.proto_source}" in '
                 f'descriptor set "{args.descriptor_set}".')

    self_ros_package = os.path.basename(os.path.dirname(args.output))
    msg_type_map = _build_msg_type_map(args.dep_descriptor_set,
                                       args.dep_mapping, args.descriptor_set,
                                       args.proto_source, self_ros_package)
    _convert(file_proto, args.output, args.proto_source, msg_type_map)


if __name__ == '__main__':
    main()
