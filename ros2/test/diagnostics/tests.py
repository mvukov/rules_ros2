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
import os
import signal
import unittest

import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import yaml

bag_dir = os.path.join(os.environ['TEST_TMPDIR'], 'bag')


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

    recorder = launch.actions.ExecuteProcess(
        cmd=[
            'ros2/test/diagnostics/bag', 'record', '-o', bag_dir,
            '/diagnostics', '/diagnostics_agg'
        ],
        output='screen',
        log_cmd=True,
    )

    return (launch.LaunchDescription([
        publisher_node,
        aggregator_node,
        recorder,
        launch.actions.TimerAction(
            period=3.0,
            actions=[
                launch.actions.EmitEvent(
                    event=launch.events.process.SignalProcess(
                        signal_number=signal.SIGINT,
                        process_matcher=lambda proc: proc is recorder))
            ]),
        launch_testing.actions.ReadyToTest(),
    ]), {
        'recorder': recorder,
    })


class TestHeartbeatDiagnostic(unittest.TestCase):

    def test_record_heartbeat_diagnostics(self, proc_info, recorder):
        proc_info.assertWaitForShutdown(process=recorder, timeout=5)
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
            process=recorder)
        bag_metadata_file = os.path.join(bag_dir, 'metadata.yaml')

        self.assertTrue(os.path.exists(bag_metadata_file))

        with open(bag_metadata_file, 'r', encoding='utf-8') as stream:
            bag_metadata = yaml.load(
                stream, Loader=yaml.Loader)['rosbag2_bagfile_information']

        min_num_received_msgs = 1
        self.assertGreaterEqual(bag_metadata['message_count'],
                                min_num_received_msgs)

        self.assertEqual(len(bag_metadata['topics_with_message_count']), 2)

        # are any heartbeat messages sent?
        diag_topic = bag_metadata['topics_with_message_count'][0]
        self.assertEqual(diag_topic['topic_metadata']['name'], '/diagnostics')
        self.assertGreaterEqual(diag_topic['message_count'],
                                min_num_received_msgs)

        # does the aggregator work (i.e. are plugins loaded)?
        agg_topic = bag_metadata['topics_with_message_count'][1]
        self.assertEqual(agg_topic['topic_metadata']['name'],
                         '/diagnostics_agg')
        self.assertGreaterEqual(agg_topic['message_count'],
                                min_num_received_msgs)
