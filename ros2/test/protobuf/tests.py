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
"""Tests for proto_to_ros2_msg.py and proto_to_ros2_converter.py."""
import sys

import pytest

from ros2.protobuf import proto_to_ros2
from ros2.protobuf import proto_to_ros2_converter
from ros2.protobuf import proto_to_ros2_msg

_PKG = 'ros2/test/protobuf'


def _ds(target: str) -> str:
    """Returns the path to a proto_library's descriptor set in Bazel runfiles.
    """
    return f'{_PKG}/{target}-descriptor-set.proto.bin'


def _load(target: str, proto_file: str):
    """Loads a FileDescriptorProto from a proto_library's descriptor set.
    """
    proto_set = proto_to_ros2.load_descriptor_set(_ds(target))
    fp = proto_to_ros2.find_file_descriptor(proto_set, f'{_PKG}/{proto_file}')
    assert fp is not None, (
        f'Proto file {proto_file!r} not found in descriptor set for {target!r}.'
        f' Files present: {[f.name for f in proto_set.file]}')
    return fp


class ConvertFieldsTests:
    """Tests that the converter correctly maps proto field types to ROS 2 types.
    """

    def test_scalar_types(self, tmp_path):
        fp = _load('scalars_proto', 'scalars.proto')
        out = tmp_path / 'Scalars.msg'
        proto_to_ros2_msg._convert(fp, str(out), f'{_PKG}/scalars.proto', {})
        content = out.read_text()
        assert 'float64 f_double' in content
        assert 'float32 f_float' in content
        assert 'int32 f_int32' in content
        assert 'int64 f_int64' in content
        assert 'uint32 f_uint32' in content
        assert 'uint64 f_uint64' in content
        assert 'int32 f_sint32' in content
        assert 'int64 f_sint64' in content
        assert 'uint32 f_fixed32' in content
        assert 'uint64 f_fixed64' in content
        assert 'int32 f_sfixed32' in content
        assert 'int64 f_sfixed64' in content
        assert 'bool f_bool' in content
        assert 'string f_string' in content
        assert 'uint8[] f_bytes' in content

    def test_repeated_scalar_fields(self, tmp_path):
        fp = _load('repeated_fields_proto', 'repeated_fields.proto')
        msg_type_map = {
            '.ros2.test.protobuf.Point':
                'ros2_test_protobuf_point_proto_ros_msgs/Point',
        }
        out = tmp_path / 'RepeatedFields.msg'
        proto_to_ros2_msg._convert(fp, str(out),
                                   f'{_PKG}/repeated_fields.proto',
                                   msg_type_map)
        content = out.read_text()
        assert 'int32[] values' in content
        assert 'float32[] scores' in content
        assert 'string[] labels' in content

    def test_repeated_message_field(self, tmp_path):
        fp = _load('repeated_fields_proto', 'repeated_fields.proto')
        msg_type_map = {
            '.ros2.test.protobuf.Point':
                'ros2_test_protobuf_point_proto_ros_msgs/Point',
        }
        out = tmp_path / 'RepeatedFields.msg'
        proto_to_ros2_msg._convert(fp, str(out),
                                   f'{_PKG}/repeated_fields.proto',
                                   msg_type_map)
        content = out.read_text()
        assert 'ros2_test_protobuf_point_proto_ros_msgs/Point[] points' in content  # noqa

    def test_message_dep_field(self, tmp_path):
        fp = _load('transform_proto', 'transform.proto')
        msg_type_map = {
            '.ros2.test.protobuf.Point':
                'ros2_test_protobuf_point_proto_ros_msgs/Point',
        }
        out = tmp_path / 'Transform.msg'
        proto_to_ros2_msg._convert(fp, str(out), f'{_PKG}/transform.proto',
                                   msg_type_map)
        content = out.read_text()
        assert 'ros2_test_protobuf_point_proto_ros_msgs/Point point' in content

    def test_sibling_proto_same_target(self, tmp_path):
        """SiblingRef references SiblingBase; both in the same proto_library."""
        fp = _load('sibling_proto', 'sibling_ref.proto')
        pkg = 'ros2_test_protobuf_sibling_proto_ros_msgs'
        msg_type_map = proto_to_ros2_msg._build_msg_type_map(
            dep_descriptor_set_paths=[],
            dep_mapping=[],
            main_descriptor_set_path=_ds('sibling_proto'),
            proto_source=f'{_PKG}/sibling_ref.proto',
            self_ros_package=pkg,
        )
        out = tmp_path / 'SiblingRef.msg'
        proto_to_ros2_msg._convert(fp, str(out), f'{_PKG}/sibling_ref.proto',
                                   msg_type_map)
        content = out.read_text()
        assert f'{pkg}/SiblingBase base' in content
        assert f'{pkg}/SiblingBase[] items' in content

    def test_message_level_enum(self, tmp_path):
        fp = _load('msg_enum_proto', 'msg_enum.proto')
        out = tmp_path / 'MsgEnum.msg'
        proto_to_ros2_msg._convert(fp, str(out), f'{_PKG}/msg_enum.proto', {})
        content = out.read_text()
        assert 'int32 KIND_NONE=0' in content
        assert 'int32 KIND_TYPE_A=1' in content
        assert 'int32 kind' in content
        assert 'int32[] extra_kinds' in content

    def test_deprecated_field_is_skipped(self, tmp_path):
        fp = _load('deprecated_field_proto', 'deprecated_field.proto')
        out = tmp_path / 'DeprecatedField.msg'
        proto_to_ros2_msg._convert(fp, str(out),
                                   f'{_PKG}/deprecated_field.proto', {})
        content = out.read_text()
        assert 'string active' in content
        assert 'bool keep_me' in content
        assert 'legacy' not in content

    def test_deprecated_unsupported_type_is_skipped(self, tmp_path):
        """Deprecated fields are skipped before constraint validation."""
        fp = _load('deprecated_map_proto', 'deprecated_map.proto')
        out = tmp_path / 'DeprecatedMap.msg'
        proto_to_ros2_msg._convert(fp, str(out), f'{_PKG}/deprecated_map.proto',
                                   {})
        content = out.read_text()
        assert 'string label' in content
        assert 'old_items' not in content


class RejectInvalidProtoTests:
    """Tests that the converter rejects each documented constraint violation."""

    def _assert_error(self, fp, proto_file, msg_type_map, fragment, tmp_path):
        with pytest.raises(SystemExit) as exc:
            proto_to_ros2_msg._convert(fp, str(tmp_path / 'out.msg'),
                                       f'{_PKG}/{proto_file}', msg_type_map)
        assert fragment in str(exc.value.code)

    def test_rejects_zero_messages(self, tmp_path):
        fp = _load('no_message_proto', 'no_message.proto')
        self._assert_error(fp, 'no_message.proto', {},
                           'expected exactly 1 message definition, got 0',
                           tmp_path)

    def test_rejects_multiple_messages(self, tmp_path):
        fp = _load('two_messages_proto', 'two_messages.proto')
        self._assert_error(fp, 'two_messages.proto', {},
                           'expected exactly 1 message definition, got 2',
                           tmp_path)

    def test_rejects_service_definition(self, tmp_path):
        fp = _load('with_service_proto', 'with_service.proto')
        self._assert_error(fp, 'with_service.proto', {},
                           'services are not supported', tmp_path)

    def test_rejects_wrong_message_name(self, tmp_path):
        fp = _load('wrong_message_name_proto', 'wrong_message_name.proto')
        self._assert_error(fp, 'wrong_message_name.proto', {},
                           'message must be named "WrongMessageName"', tmp_path)

    def test_rejects_nested_message_type(self, tmp_path):
        fp = _load('nested_message_proto', 'nested_message.proto')
        self._assert_error(fp, 'nested_message.proto', {},
                           'nested message type', tmp_path)

    def test_rejects_oneof_field(self, tmp_path):
        fp = _load('oneof_field_proto', 'oneof_field.proto')
        self._assert_error(fp, 'oneof_field.proto', {}, 'oneof', tmp_path)

    def test_rejects_map_field(self, tmp_path):
        fp = _load('map_field_proto', 'map_field.proto')
        self._assert_error(fp, 'map_field.proto', {}, 'map field', tmp_path)

    def test_rejects_repeated_bytes(self, tmp_path):
        fp = _load('repeated_bytes_proto', 'repeated_bytes.proto')
        self._assert_error(fp, 'repeated_bytes.proto', {}, 'repeated bytes',
                           tmp_path)

    def test_rejects_cross_file_enum(self, tmp_path):
        fp = _load('foreign_enum_proto', 'foreign_enum_usage.proto')
        self._assert_error(fp, 'foreign_enum_usage.proto', {},
                           'not found in the current proto file', tmp_path)

    def test_rejects_file_level_enum(self, tmp_path):
        fp = _load('file_enum_proto', 'file_enum.proto')
        self._assert_error(fp, 'file_enum.proto', {},
                           'file-level enum definitions are not supported',
                           tmp_path)

    def test_rejects_missing_dep_mapping(self, tmp_path):
        fp = _load('transform_proto', 'transform.proto')
        self._assert_error(fp, 'transform.proto', {}, 'no dep_mapping entry',
                           tmp_path)


class ConverterSkipsDeprecatedTests:
    """Tests that the C++ converter generator skips deprecated fields."""

    def _field_lines(self, target, proto_file):
        fp = _load(target, proto_file)
        message = fp.message_type[0]
        to_ros, from_ros, _ = proto_to_ros2_converter._field_conversions(
            message, f'{_PKG}/{proto_file}', {})
        return to_ros, from_ros

    def test_deprecated_scalar_not_in_generated_code(self):
        to_ros, from_ros = self._field_lines('deprecated_field_proto',
                                             'deprecated_field.proto')
        all_lines = to_ros + from_ros
        assert any('active' in line for line in all_lines)
        assert any('keep_me' in line for line in all_lines)
        assert not any('legacy' in line for line in all_lines)


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v', *sys.argv[1:]]))
