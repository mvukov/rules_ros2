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
"""Generates C++ proto<->ROS2 converter header and source from a proto file.

For each proto message the tool emits two free functions in namespace
``<ros_package_name>::proto_converters``:

  <RosType> ToRos(const <ProtoType>& proto);
  <ProtoType> FromRos(const <RosType>& ros);

Limitations mirror those of proto_to_ros2_msg.py:
- Exactly one message definition per proto file.
- Service definitions are not supported.
- Message-type fields require a --dep_mapping entry so the ROS2 package can
  be resolved.
- Enum and group fields are not supported.
- Repeated bytes fields are not supported.
"""
import argparse
import io
import re
import sys

import em
from google.protobuf import descriptor_pb2
from google.protobuf.descriptor_pb2 import FieldDescriptorProto

# ---------------------------------------------------------------------------
# Field-type tables
# ---------------------------------------------------------------------------

# Proto scalar types → C++ type used in ROS2 structs.
_SCALAR_CPP_TYPE = {
    FieldDescriptorProto.TYPE_DOUBLE: 'double',
    FieldDescriptorProto.TYPE_FLOAT: 'float',
    FieldDescriptorProto.TYPE_INT32: 'int32_t',
    FieldDescriptorProto.TYPE_INT64: 'int64_t',
    FieldDescriptorProto.TYPE_UINT32: 'uint32_t',
    FieldDescriptorProto.TYPE_UINT64: 'uint64_t',
    FieldDescriptorProto.TYPE_SINT32: 'int32_t',
    FieldDescriptorProto.TYPE_SINT64: 'int64_t',
    FieldDescriptorProto.TYPE_FIXED32: 'uint32_t',
    FieldDescriptorProto.TYPE_FIXED64: 'uint64_t',
    FieldDescriptorProto.TYPE_SFIXED32: 'int32_t',
    FieldDescriptorProto.TYPE_SFIXED64: 'int64_t',
    FieldDescriptorProto.TYPE_BOOL: 'bool',
}

_UNSUPPORTED_TYPES = {
    FieldDescriptorProto.TYPE_GROUP: 'group',
    FieldDescriptorProto.TYPE_ENUM: 'enum',
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _to_snake_case(name):
    """Converts CamelCase to snake_case (mirrors rosidl_cmake convention)."""
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    s2 = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1)
    return s2.lower()


def _proto_package_to_ns(package):
    """Converts 'a.b.c' → 'a::b::c'."""
    return '::'.join(package.split('.')) if package else ''


def _build_fqn_to_dep_pkg_map(dep_descriptor_set_paths, dep_mapping):
    """Build {'.pkg.MsgName': 'dep_ros_package_name'} from dep descriptor sets.
    """
    path_to_ros_pkg = {}
    for entry in dep_mapping:
        proto_path, ros2_pkg = entry.split(':', 1)
        path_to_ros_pkg[proto_path] = ros2_pkg

    fqn_map = {}
    for ds_path in dep_descriptor_set_paths:
        with open(ds_path, 'rb') as f:
            dep_set = descriptor_pb2.FileDescriptorSet()
            dep_set.ParseFromString(f.read())
        for file_proto in dep_set.file:
            ros2_pkg = path_to_ros_pkg.get(file_proto.name)
            if ros2_pkg is None:
                continue
            pkg_prefix = '.' + file_proto.package if file_proto.package else ''
            for msg in file_proto.message_type:
                fqn = f'{pkg_prefix}.{msg.name}'
                fqn_map[fqn] = ros2_pkg
    return fqn_map


def _find_file_descriptor(proto_set, proto_source):
    """Find a FileDescriptorProto by name, with basename fallback."""
    for fp in proto_set.file:
        if fp.name == proto_source:
            return fp
    source_base = proto_source.split('/')[-1]
    for fp in proto_set.file:
        if fp.name.split('/')[-1] == source_base:
            return fp
    return None


# ---------------------------------------------------------------------------
# Per-field conversion code generation
# ---------------------------------------------------------------------------


def _field_conversions(message, proto_source, fqn_map):
    """Return (to_ros_lines, from_ros_lines, dep_pkgs_used) for one message.

    Each entry in to_ros_lines / from_ros_lines is a C++ statement string
    (already indented with two spaces).  dep_pkgs_used is the set of dep
    ros_package_names whose converters are called.
    """
    to_ros = []
    from_ros = []
    dep_pkgs = set()

    for field in message.field:
        name = field.name
        is_repeated = field.label == FieldDescriptorProto.LABEL_REPEATED
        ftype = field.type

        if ftype in _UNSUPPORTED_TYPES:
            sys.exit(f'Error: {proto_source}: field "{name}": '
                     f'{_UNSUPPORTED_TYPES[ftype]} fields are not supported.')

        # ---- bytes ----------------------------------------------------------
        if ftype == FieldDescriptorProto.TYPE_BYTES:
            if is_repeated:
                sys.exit(f'Error: {proto_source}: field "{name}": '
                         f'repeated bytes is not supported.')
            to_ros.append(f'  ros.{name} = std::vector<uint8_t>'
                          f'(proto.{name}().begin(), proto.{name}().end());')
            from_ros.append(
                f'  proto.set_{name}'
                f'(std::string(ros.{name}.begin(), ros.{name}.end()));')
            continue

        # ---- string ---------------------------------------------------------
        if ftype == FieldDescriptorProto.TYPE_STRING:
            if is_repeated:
                to_ros.append(
                    f'  ros.{name} = std::vector<std::string>'
                    f'(proto.{name}().begin(), proto.{name}().end());')
                from_ros.append(f'  for (const auto& s : ros.{name}) {{')
                from_ros.append(f'    proto.add_{name}(s);')
                from_ros.append('  }')
            else:
                to_ros.append(f'  ros.{name} = proto.{name}();')
                from_ros.append(f'  proto.set_{name}(ros.{name});')
            continue

        # ---- message --------------------------------------------------------
        if ftype == FieldDescriptorProto.TYPE_MESSAGE:
            dep_pkg = fqn_map.get(field.type_name)
            if dep_pkg is None:
                sys.exit(
                    f'Error: {proto_source}: field "{name}" references '
                    f'message type "{field.type_name}" with no dep_mapping '
                    f'entry.  Add a --dep_mapping for the proto file that '
                    f'defines it.')
            dep_pkgs.add(dep_pkg)
            conv = f'{dep_pkg}::proto_converters'

            if is_repeated:
                to_ros.append(f'  for (const auto& item : proto.{name}()) {{')
                to_ros.append(f'    ros.{name}.push_back({conv}::ToRos(item));')
                to_ros.append('  }')
                from_ros.append(f'  for (const auto& item : ros.{name}) {{')
                from_ros.append(
                    f'    *proto.add_{name}() = {conv}::FromRos(item);')
                from_ros.append('  }')
            else:
                to_ros.append(f'  ros.{name} = {conv}::ToRos(proto.{name}());')
                from_ros.append(f'  *proto.mutable_{name}() = '
                                f'{conv}::FromRos(ros.{name});')
            continue

        # ---- scalar (all remaining types) -----------------------------------
        if ftype not in _SCALAR_CPP_TYPE:
            sys.exit(f'Error: {proto_source}: field "{name}" has unknown '
                     f'field type value {ftype}.')

        cpp_type = _SCALAR_CPP_TYPE[ftype]

        if is_repeated:
            to_ros.append(f'  ros.{name} = std::vector<{cpp_type}>'
                          f'(proto.{name}().begin(), proto.{name}().end());')
            from_ros.append(f'  proto.mutable_{name}()->Assign'
                            f'(ros.{name}.begin(), ros.{name}.end());')
        else:
            to_ros.append(f'  ros.{name} = proto.{name}();')
            from_ros.append(f'  proto.set_{name}(ros.{name});')

    return to_ros, from_ros, dep_pkgs


# ---------------------------------------------------------------------------
# Empy templates
# ---------------------------------------------------------------------------

_HEADER_TEMPLATE = """\
// Generated by proto_to_ros2_converter. Do not edit.
#pragma once

@(ctx['includes_section'])

namespace @(ctx['ros_package_name'])::proto_converters {

@[for msg in ctx['messages']]
@(msg['ros_type']) ToRos(const @(msg['proto_type'])& proto);

@(msg['proto_type']) FromRos(const @(msg['ros_type'])& ros);
@[end for]

}  // namespace @(ctx['ros_package_name'])::proto_converters
"""

_SOURCE_TEMPLATE = """\
// Generated by proto_to_ros2_converter. Do not edit.
#include "@(ctx['ros_package_name'])/proto_converters.h"
@[for inc in ctx['dep_converter_includes']]
#include "@(inc)"
@[end for]

namespace @(ctx['ros_package_name'])::proto_converters {

@[for msg in ctx['messages']]
@(msg['ros_type']) ToRos(const @(msg['proto_type'])& proto) {
  @(msg['ros_type']) ros;
@(msg['to_ros_body'])
  return ros;
}

@(msg['proto_type']) FromRos(const @(msg['ros_type'])& ros) {
  @(msg['proto_type']) proto;
@(msg['from_ros_body'])
  return proto;
}
@[end for]

}  // namespace @(ctx['ros_package_name'])::proto_converters
"""


def _render(template, context):
    """Render an empy 3.3 template string with a 'ctx' variable."""
    output = io.StringIO()
    interp = em.Interpreter(output=output, globals={'ctx': context})
    try:
        interp.string(template)
        return output.getvalue()
    finally:
        interp.shutdown()


# ---------------------------------------------------------------------------
# Main conversion logic
# ---------------------------------------------------------------------------


def _convert(descriptor_set, proto_sources, ros_package_name, fqn_map,
             output_header, output_source):
    """Generate converter .h and .cc for all proto sources in the descriptor."""
    messages = []
    dep_pkgs_all = set()

    proto_includes = []
    ros2_includes = []

    for proto_source in proto_sources:
        file_proto = _find_file_descriptor(descriptor_set, proto_source)
        if file_proto is None:
            sys.exit(f'Error: could not find proto source "{proto_source}" in '
                     f'the descriptor set.')

        if file_proto.service:
            sys.exit(f'Error: {proto_source}: services are not supported.')

        num_msgs = len(file_proto.message_type)
        if num_msgs != 1:
            sys.exit(f'Error: {proto_source}: expected exactly 1 message '
                     f'definition, got {num_msgs}.')

        message = file_proto.message_type[0]
        msg_name = message.name

        proto_ns = _proto_package_to_ns(file_proto.package)
        proto_type = f'{proto_ns}::{msg_name}' if proto_ns else msg_name
        ros_type = f'{ros_package_name}::msg::{msg_name}'

        # Proto C++ include: replace .proto suffix with .pb.h
        proto_include = file_proto.name[:-len('.proto')] + '.pb.h'
        # ROS2 msg include: snake_case name with .hpp extension
        ros2_include = (f'{ros_package_name}/msg/'
                        f'{_to_snake_case(msg_name)}.hpp')

        to_ros_lines, from_ros_lines, dep_pkgs = _field_conversions(
            message, proto_source, fqn_map)
        dep_pkgs_all.update(dep_pkgs)

        to_ros_body = '\n'.join(to_ros_lines)
        from_ros_body = '\n'.join(from_ros_lines)

        messages.append({
            'msg_name': msg_name,
            'proto_type': proto_type,
            'ros_type': ros_type,
            'to_ros_body': to_ros_body,
            'from_ros_body': from_ros_body,
        })

        proto_includes.append(proto_include)
        ros2_includes.append(ros2_include)

    # Build a sorted, deduplicated dep-converter include list.
    dep_converter_includes = sorted(
        f'{pkg}/proto_converters.h' for pkg in dep_pkgs_all)

    # Header includes: only the current package's proto and ROS2 msg types.
    # Dep converter includes go into the .cc file, not the header.
    include_lines = []
    for inc in proto_includes:
        include_lines.append(f'#include "{inc}"')
    include_lines.append('')
    for inc in ros2_includes:
        include_lines.append(f'#include "{inc}"')
    includes_section = '\n'.join(include_lines)

    header_context = {
        'ros_package_name': ros_package_name,
        'includes_section': includes_section,
        'messages': messages,
    }
    source_context = {
        'ros_package_name': ros_package_name,
        'dep_converter_includes': dep_converter_includes,
        'messages': messages,
    }

    with open(output_header, 'w') as f:
        f.write(_render(_HEADER_TEMPLATE, header_context))
    with open(output_source, 'w') as f:
        f.write(_render(_SOURCE_TEMPLATE, source_context))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description='Generate C++ proto<->ROS2 converter code.')
    parser.add_argument(
        '--descriptor_set',
        required=True,
        help='Path to the binary FileDescriptorSet file.',
    )
    parser.add_argument(
        '--ros_package_name',
        required=True,
        help='ROS2 package name (e.g. point_proto_ros_msgs).',
    )
    parser.add_argument(
        '--output_header',
        required=True,
        help='Path of the output .h file to write.',
    )
    parser.add_argument(
        '--output_source',
        required=True,
        help='Path of the output .cc file to write.',
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
        proto_set = descriptor_pb2.FileDescriptorSet()
        proto_set.ParseFromString(f.read())

    fqn_map = _build_fqn_to_dep_pkg_map(args.dep_descriptor_set,
                                        args.dep_mapping)

    proto_sources = [fp.name for fp in proto_set.file]

    _convert(
        descriptor_set=proto_set,
        proto_sources=proto_sources,
        ros_package_name=args.ros_package_name,
        fqn_map=fqn_map,
        output_header=args.output_header,
        output_source=args.output_source,
    )


if __name__ == '__main__':
    main()
