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
import tf2_py


def test_buffer_core_creation():
    buffer = tf2_py.BufferCore()
    assert buffer is not None


def test_exception_hierarchy():
    assert issubclass(tf2_py.LookupException, tf2_py.TransformException)
    assert issubclass(tf2_py.ConnectivityException, tf2_py.TransformException)
    assert issubclass(tf2_py.ExtrapolationException, tf2_py.TransformException)
    assert issubclass(tf2_py.InvalidArgumentException,
                      tf2_py.TransformException)


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
