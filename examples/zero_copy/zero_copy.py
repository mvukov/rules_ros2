# Copyright 2022 Milan Vukov
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
"""Launch a talker and a listener."""
import launch.actions
import launch_ros.actions
import zero_copy.roudi  # This is an auto-generated file, see BUILD.bazel file.


def generate_launch_description():
    """Launch a talker and a listener."""
    return launch.LaunchDescription([
        # roudi is a shared memory manager that also has to be started.
        launch.actions.ExecuteProcess(
            name='iceoryx_roudi',
            cmd=[zero_copy.roudi.ROUDI_PATH],
        ),
        launch.actions.SetEnvironmentVariable(name='CYCLONEDDS_URI',
                                              value='zero_copy/cyclonedds.xml'),
        launch_ros.actions.Node(executable='zero_copy/talker',
                                output='screen',
                                name='talker'),
        launch_ros.actions.Node(executable='zero_copy/listener',
                                output='screen',
                                name='listener'),
    ])
