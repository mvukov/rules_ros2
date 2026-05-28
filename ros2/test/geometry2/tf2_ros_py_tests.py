# Copyright 2026 Milan Vukov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import pytest
import rclpy.time
import tf2_ros
from geometry_msgs.msg import TransformStamped


def _make_identity_transform(parent_frame: str,
                             child_frame: str) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.rotation.w = 1.0
    return t


def test_buffer_creation():
    buffer = tf2_ros.Buffer()
    assert buffer is not None


def test_can_transform_returns_false_for_unknown_frames():
    buffer = tf2_ros.Buffer()
    assert not buffer.can_transform('unknown_parent', 'unknown_child',
                                    rclpy.time.Time())


def test_set_and_lookup_transform():
    buffer = tf2_ros.Buffer()
    transform = _make_identity_transform('base', 'child')
    buffer.set_transform(transform, 'test')

    result = buffer.lookup_transform('base', 'child', rclpy.time.Time())
    assert result.header.frame_id == 'base'
    assert result.child_frame_id == 'child'


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
