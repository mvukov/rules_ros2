# Copyright 2018 Open Source Robotics Foundation, Inc.
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
"""Launch lifecycle nodes."""
import os

import launch.actions
import launch_ros.actions


def generate_launch_description():
    """Launch lifecycle nodes."""
    # Robustly find the executable relative to this launch file
    dir_path = os.path.dirname(os.path.abspath(__file__))
    talker_exec = os.path.join(dir_path, 'lifecycle_talker')
    listener_exec = os.path.join(dir_path, 'lifecycle_listener')
    client_exec = os.path.join(dir_path, 'lifecycle_service_client')

    return launch.LaunchDescription([
        launch_ros.actions.LifecycleNode(
            executable=talker_exec,
            name='lc_talker',
            namespace='',
            output='screen'),
        launch_ros.actions.Node(executable=listener_exec,
                                output='screen'),
        launch_ros.actions.Node(executable=client_exec,
                                output='screen',
                                on_exit=launch.actions.Shutdown()),
    ])
