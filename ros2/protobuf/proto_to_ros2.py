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
"""Shared utilities for proto-to-ROS 2 code generation tools."""
from google.protobuf import descriptor_pb2
from google.protobuf.descriptor_pb2 import FieldDescriptorProto

# Non-scalar proto field types that are explicitly unsupported.
UNSUPPORTED_TYPES = {
    FieldDescriptorProto.TYPE_GROUP: 'group',
}


def load_descriptor_set(path):
    """Loads and parses a binary FileDescriptorSet from a file path."""
    with open(path, 'rb') as f:
        proto_set = descriptor_pb2.FileDescriptorSet()
        proto_set.ParseFromString(f.read())
    return proto_set


def find_file_descriptor(proto_set, proto_source):
    """Finds a FileDescriptorProto by name, with fallback to basename
       matching.
    """
    for file_proto in proto_set.file:
        if file_proto.name == proto_source:
            return file_proto
    source_basename = proto_source.split('/')[-1]
    for file_proto in proto_set.file:
        if file_proto.name.split('/')[-1] == source_basename:
            return file_proto
    return None


def parse_dep_mapping(dep_mapping):
    """Parses a list of 'proto_path:ros_package' strings into a dict."""
    return dict(entry.split(':', 1) for entry in dep_mapping)
