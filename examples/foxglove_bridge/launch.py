# Copyright 2023 Milan Vukov
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
import launch.actions
import launch_ros.actions
from foxglove_bridge import data_paths

import third_party.foxglove_bridge.node_path
import third_party.foxglove_bridge.params


def generate_launch_description():
    with open(data_paths.SAM_BOT_URDF_PATH, 'r', encoding='utf-8') as stream:
        sam_bot_urdf = stream.read()

    return launch.LaunchDescription([
        # ROS_DISTRO is necessary for correct operation of the Foxglove Studio.
        launch.actions.SetEnvironmentVariable(name='ROS_DISTRO',
                                              value='humble'),
        launch_ros.actions.Node(
            executable=data_paths.ROBOT_STATE_PUBLISHER_NODE_PATH,
            output='screen',
            parameters=[{
                'robot_description': sam_bot_urdf,
            }],
        ),
        launch_ros.actions.Node(
            executable=third_party.foxglove_bridge.node_path.NODE_PATH,
            output='screen',
            parameters=[
                third_party.foxglove_bridge.params.PARAMS_TO_DEFAULT_VALUES,
            ],
        ),
        launch_ros.actions.Node(
            executable='foxglove_bridge/publisher',
            output='screen',
        ),
    ])
