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
import launch.substitutions
import launch_ros.actions
import zero_copy.roudi  # This is an auto-generated file, see BUILD.bazel file.

CALLBACK_PERIOD_MS_ARG = 'callback_period_ms'
TALKER_TYPE_ARG = 'talker_type'
LISTENER_TYPE_ARG = 'listener_type'


def set_up_nodes(context, callback_period_ms_arg, talker_type_arg,
                 listener_type_arg):
    callback_period_ms_str = context.perform_substitution(
        callback_period_ms_arg)
    talker_type = context.perform_substitution(talker_type_arg)
    listener_type = context.perform_substitution(listener_type_arg)

    match talker_type:
        case 'cc':
            talker_exe = 'zero_copy/talker'
        case 'rust':
            talker_exe = 'zero_copy/rust_talker'
        case _:
            raise ValueError(f'{TALKER_TYPE_ARG} must be `cc` or `rust`!')

    match listener_type:
        case 'cc':
            listener_exe = 'zero_copy/listener'
        case 'rust':
            listener_exe = 'zero_copy/rust_listener'
        case _:
            raise ValueError(f'{LISTENER_TYPE_ARG} must be `cc` or `rust`!')

    return [
        launch_ros.actions.Node(
            executable=talker_exe,
            output='screen',
            name='talker',
            parameters=[{
                CALLBACK_PERIOD_MS_ARG: int(callback_period_ms_str)
            }],
        ),
        launch_ros.actions.Node(executable=listener_exe,
                                output='screen',
                                name='listener')
    ]


def generate_launch_description():
    """Launch a talker and a listener."""
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(CALLBACK_PERIOD_MS_ARG,
                                             default_value='100'),
        launch.actions.DeclareLaunchArgument(TALKER_TYPE_ARG,
                                             default_value='cc'),
        launch.actions.DeclareLaunchArgument(LISTENER_TYPE_ARG,
                                             default_value='cc'),
        # roudi is a shared memory manager that also has to be started.
        launch.actions.ExecuteProcess(
            name='iceoryx_roudi',
            cmd=[zero_copy.roudi.ROUDI_PATH],
        ),
        launch.actions.SetEnvironmentVariable(name='CYCLONEDDS_URI',
                                              value='zero_copy/cyclonedds.xml'),
        # https://github.com/ros2/rmw_cyclonedds/issues/469#issuecomment-1877574593
        launch.actions.SetEnvironmentVariable(
            name='ROS_DISABLE_LOANED_MESSAGES', value='0'),
        launch.actions.OpaqueFunction(
            function=set_up_nodes,
            args=[
                launch.substitutions.LaunchConfiguration(
                    CALLBACK_PERIOD_MS_ARG),
                launch.substitutions.LaunchConfiguration(TALKER_TYPE_ARG),
                launch.substitutions.LaunchConfiguration(LISTENER_TYPE_ARG),
            ],
        ),
    ])
