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
import launch_testing.actions
import launch_testing.asserts


def generate_test_description():
    plugin_tests_node = launch.actions.ExecuteProcess(
        name='plugin_tests',
        cmd=['ros2/test/pluginlib/plugin_tests'],
    )

    return (launch.LaunchDescription([
        plugin_tests_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'plugin_tests': plugin_tests_node,
    })


class TestPlugins(unittest.TestCase):

    def test_plugin_tests(self, proc_info, plugin_tests):
        proc_info.assertWaitForShutdown(process=plugin_tests, timeout=2)
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
            process=plugin_tests)
