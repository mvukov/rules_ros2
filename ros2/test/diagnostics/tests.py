# Copyright 2023 Dennis Bruggner
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
import threading
import unittest

import diagnostic_msgs.msg
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy


@launch_testing.markers.keep_alive
def generate_test_description():
    publisher_node = launch_ros.actions.Node(
        executable='ros2/test/diagnostics/diagnostic_publisher',
        name='diagnostic_publisher',
        output='screen',
    )

    aggregator_node = launch_ros.actions.Node(
        executable='../ros2_diagnostics/aggregator_node',
        name='diagnostic_aggregator',
        parameters=['ros2/test/diagnostics/aggregator_config.yaml'],
    )

    return launch.LaunchDescription([
        publisher_node,
        aggregator_node,
        launch_testing.actions.ReadyToTest(),
    ])


class DiagnosticsListener(rclpy.node.Node):

    def __init__(self):
        super().__init__('diagnostics_listener')

        self.diagnostics_subscription = self.create_subscription(
            diagnostic_msgs.msg.DiagnosticArray, '/diagnostics',
            self._on_diagnostics, 10)
        self.diagnostics_agg_subscription = self.create_subscription(
            diagnostic_msgs.msg.DiagnosticArray, '/diagnostics_agg',
            self._on_diagnostics_agg, 10)
        self.messages = {}
        self.both_messages_received = threading.Event()

    def _on_diagnostics(self, msg):
        self.messages['/diagnostics'] = msg
        self._check_if_both_messages_received()

    def _on_diagnostics_agg(self, msg):
        self.messages['/diagnostics_agg'] = msg
        self._check_if_both_messages_received()

    def _check_if_both_messages_received(self):
        if len(self.messages) == 2:
            self.both_messages_received.set()


class TestHeartbeatDiagnostic(unittest.TestCase):

    def test_record_heartbeat_diagnostics(self):
        rclpy.init()
        try:
            # Start listener node and wait for messages to be received.
            diagnostics_listener = DiagnosticsListener()
            thread = threading.Thread(target=lambda node: rclpy.spin(node),
                                      args=(diagnostics_listener,))
            thread.start()
            event_triggered = diagnostics_listener.both_messages_received.wait(
                timeout=10.0)
            self.assertTrue(event_triggered,
                            'timeout while waiting for messages')

            # Basic sanity checks of messages.
            diagnostics_msg = diagnostics_listener.messages['/diagnostics']
            self.assertEqual(diagnostics_msg.status[0].message, 'Alive')

            diagnostics_agg_msg = diagnostics_listener.messages[
                '/diagnostics_agg']
            status_names = (
                status.name for status in diagnostics_agg_msg.status)
            self.assertTrue(
                any('diagnostic_publisher' in name for name in status_names))
        finally:
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestHeartbeatDiagnosticShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
