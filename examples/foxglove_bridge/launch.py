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
import os

import launch.actions
import launch_ros.actions

import third_party.foxglove_bridge.node_path
import third_party.foxglove_bridge.params

try:
    from examples.foxglove_bridge import data_paths
except ImportError:
    from foxglove_bridge import data_paths


def get_abs_path(path):
    """Resolve a path relative to the runfiles directory if it's not absolute."""
    if os.path.isabs(path):
        return path
    runfiles_dir = os.environ.get('RUNFILES_DIR')
    if runfiles_dir:
        return os.path.join(runfiles_dir, path)
    # Fallback: this might fail if RUNFILES_DIR is not set, but it's required for bazel run
    return path


def generate_launch_description():
    # Resolve paths
    sam_bot_urdf_abs_path = get_abs_path(data_paths.SAM_BOT_URDF_PATH)
    robot_state_publisher_node_abs_path = get_abs_path(data_paths.ROBOT_STATE_PUBLISHER_NODE_PATH)
    foxglove_bridge_node_abs_path = get_abs_path(third_party.foxglove_bridge.node_path.NODE_PATH)
    
    # Publisher is in the same directory as this launch file
    dir_path = os.path.dirname(os.path.abspath(__file__))
    publisher_abs_path = os.path.join(dir_path, 'publisher')

    with open(sam_bot_urdf_abs_path, encoding='utf-8') as stream:
        sam_bot_urdf = stream.read()

    return launch.LaunchDescription([
        # ROS_DISTRO is necessary for correct operation of the Foxglove Studio.
        launch.actions.SetEnvironmentVariable(name='ROS_DISTRO',
                                              value='humble'),
        launch_ros.actions.Node(
            executable=robot_state_publisher_node_abs_path,
            output='screen',
            parameters=[{
                'robot_description': sam_bot_urdf,
            }],
        ),
        launch_ros.actions.Node(
            executable=foxglove_bridge_node_abs_path,
            output='screen',
            parameters=[
                third_party.foxglove_bridge.params.PARAMS_TO_DEFAULT_VALUES,
            ],
        ),
        launch_ros.actions.Node(
            executable=publisher_abs_path,
            output='screen',
        ),
    ])