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
import launch
import launch_pytest.tools
import pytest


@pytest.fixture
def static_transform_publisher_proc():
    return launch.actions.ExecuteProcess(
        cmd=[
            'ros2/test/geometry2/static_transform_publisher',
            '--frame-id',
            'world',
            '--child-frame-id',
            'base',
        ],
        cached_output=True,
    )


@pytest.fixture
def tf2_echo_proc():
    return launch.actions.ExecuteProcess(
        cmd=['ros2/test/geometry2/tf2_echo', 'world', 'base'],
        cached_output=True,
    )


@launch_pytest.fixture
def launch_description(static_transform_publisher_proc, tf2_echo_proc):
    return launch.LaunchDescription([
        static_transform_publisher_proc,
        tf2_echo_proc,
        launch_pytest.actions.ReadyToTest(),
    ])


@pytest.mark.launch(fixture=launch_description)
def test_nodes_start(static_transform_publisher_proc, tf2_echo_proc,
                     launch_context):
    launch_pytest.tools.wait_for_start_sync(launch_context,
                                            static_transform_publisher_proc,
                                            timeout=5)
    launch_pytest.tools.wait_for_start_sync(launch_context,
                                            tf2_echo_proc,
                                            timeout=5)
