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
import launch.actions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.LifecycleNode(
            executable='lifecycle/lifecycle_talker',
            name='lc_talker',
            namespace='',
            output='screen'),
        launch_ros.actions.Node(executable='lifecycle/lifecycle_listener',
                                output='screen'),
        launch_ros.actions.Node(executable='lifecycle/lifecycle_service_client',
                                output='screen',
                                on_exit=launch.actions.Shutdown()),
    ])
