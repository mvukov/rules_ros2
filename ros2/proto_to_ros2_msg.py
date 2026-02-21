# Copyright 2024 Milan Vukov
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
- Only proto3 scalar field types are supported (no message-type fields, no enum
  fields).
- Repeated scalar fields are supported and map to dynamic arrays (e.g.
  `int32[] values`).
- proto `bytes` fields map to `uint8[]` in ROS2.
"""
import argparse
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
    FieldDescriptorProto.TYPE_MESSAGE: 'message',
    FieldDescriptorProto.TYPE_GROUP: 'group',
    FieldDescriptorProto.TYPE_ENUM: 'enum',
}


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


def _convert(file_proto, output_path, proto_source):
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

        if field_type_value in _UNSUPPORTED_TYPES:
            type_name = _UNSUPPORTED_TYPES[field_type_value]
            sys.exit(
                f'Error: {proto_source}: field "{field.name}" has unsupported '
                f'type "{type_name}". Only scalar types are supported.')

        if field_type_value not in _PROTO_TO_ROS2_TYPE:
            sys.exit(f'Error: {proto_source}: field "{field.name}" has unknown '
                     f'type value {field_type_value}.')

        ros2_type = _PROTO_TO_ROS2_TYPE[field_type_value]
        is_repeated = (field.label == FieldDescriptorProto.LABEL_REPEATED)

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
    args = parser.parse_args()

    with open(args.descriptor_set, 'rb') as f:
        data = f.read()

    proto_set = descriptor_pb2.FileDescriptorSet()
    proto_set.ParseFromString(data)

    file_proto = _find_file_descriptor(proto_set, args.proto_source)
    if file_proto is None:
        sys.exit(f'Error: could not find proto source "{args.proto_source}" in '
                 f'descriptor set "{args.descriptor_set}".')

    _convert(file_proto, args.output, args.proto_source)


if __name__ == '__main__':
    main()
