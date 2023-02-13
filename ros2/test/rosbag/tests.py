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
import os
import unittest

import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import yaml


@launch_testing.markers.keep_alive
def generate_test_description():
    publisher_node = launch_ros.actions.Node(
        executable='ros2/test/rosbag/publisher',
        output='screen',
    )

    recorder_node = launch_ros.actions.Node(
        executable='ros2/test/rosbag/recorder',
        output='screen',
    )

    return (launch.LaunchDescription([
        publisher_node,
        recorder_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'recorder': recorder_node,
    })


class TestRecorder(unittest.TestCase):

    def test_recorder(self, proc_info, recorder):
        proc_info.assertWaitForShutdown(process=recorder, timeout=5)
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
            process=recorder)
        bag_dir = os.path.join(os.environ['TEST_UNDECLARED_OUTPUTS_DIR'], 'bag')
        bag_db_file = os.path.join(bag_dir, 'bag_0.db3')
        bag_metadata_file = os.path.join(bag_dir, 'metadata.yaml')

        self.assertTrue(os.path.exists(bag_db_file))
        self.assertTrue(os.path.exists(bag_metadata_file))

        with open(bag_metadata_file, 'r', encoding='utf-8') as stream:
            bag_metadata = yaml.load(
                stream, Loader=yaml.Loader)['rosbag2_bagfile_information']

        min_num_received_msgs = 9
        self.assertGreaterEqual(bag_metadata['message_count'],
                                min_num_received_msgs)

        topic = bag_metadata['topics_with_message_count'][0]
        self.assertEqual(topic['topic_metadata']['name'], '/topic')
        self.assertGreaterEqual(topic['message_count'], min_num_received_msgs)
