# Copyright 2022 wayve.ai
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
import unittest

import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import zero_copy.roudi


def generate_test_description():
    talker_tests_node = launch_ros.actions.Node(
        executable='zero_copy/talker_tests',
        output='screen',
    )

    return (launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            name='iceoryx_roudi',
            cmd=[zero_copy.roudi.ROUDI_PATH],
        ),
        launch.actions.SetEnvironmentVariable(name='CYCLONEDDS_URI',
                                              value='zero_copy/cyclonedds.xml'),
        launch_ros.actions.Node(executable='zero_copy/talker',
                                parameters=[{
                                    'callback_period_ms': 10
                                }],
                                arguments=['--ros-args', '--log-level',
                                           'WARN']),
        talker_tests_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'talker_tests': talker_tests_node,
    })


class TestTalker(unittest.TestCase):

    def test_talker_tests(self, proc_info, talker_tests):
        proc_info.assertWaitForShutdown(process=talker_tests, timeout=5)
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
            process=talker_tests)
