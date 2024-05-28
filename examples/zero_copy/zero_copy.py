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


def set_up_talker(context, callback_period_ms_arg, talker_type_arg):
    callback_period_ms_str = context.perform_substitution(
        callback_period_ms_arg)
    talker_type = context.perform_substitution(talker_type_arg)

    match talker_type:
        case 'cc':
            exe = 'zero_copy/talker'
        case 'rust':
            exe = 'zero_copy/rust_talker'
        case _:
            raise ValueError(f'{TALKER_TYPE_ARG} must be `cc` or `rust`!')

    return [
        launch_ros.actions.Node(
            executable=exe,
            output='screen',
            name='talker',
            parameters=[{
                CALLBACK_PERIOD_MS_ARG: int(callback_period_ms_str)
            }],
        )
    ]


def generate_launch_description():
    """Launch a talker and a listener."""
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(CALLBACK_PERIOD_MS_ARG,
                                             default_value='100'),
        launch.actions.DeclareLaunchArgument(TALKER_TYPE_ARG,
                                             default_value='cc'),
        # roudi is a shared memory manager that also has to be started.
        launch.actions.ExecuteProcess(
            name='iceoryx_roudi',
            cmd=[zero_copy.roudi.ROUDI_PATH],
        ),
        launch.actions.SetEnvironmentVariable(name='CYCLONEDDS_URI',
                                              value='zero_copy/cyclonedds.xml'),
        launch.actions.OpaqueFunction(
            function=set_up_talker,
            args=[
                launch.substitutions.LaunchConfiguration(
                    CALLBACK_PERIOD_MS_ARG),
                launch.substitutions.LaunchConfiguration(TALKER_TYPE_ARG),
            ],
        ),
        launch_ros.actions.Node(executable='zero_copy/listener',
                                output='screen',
                                name='listener'),
    ])
