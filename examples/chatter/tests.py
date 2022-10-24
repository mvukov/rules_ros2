# Copyright 2019 Apex.AI, Inc.
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
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
import std_msgs.msg


def generate_test_description():
    talker_node = launch_ros.actions.Node(executable='chatter/talker',
                                          parameters=[
                                              {
                                                  'callback_period_ms': 10
                                              },
                                          ])

    return (
        launch.LaunchDescription([
            talker_node,
            # Start tests right away - no need to wait for anything.
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'talker': talker_node,
        })


class TestTalkerListenerLink(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_talker_listener_link')

    def tearDown(self):
        self.node.destroy_node()

    def test_talker_transmits(self, launch_service, talker, proc_output):
        # Expect the talker to publish strings on '/topic'.
        msgs_rx = []

        sub = self.node.create_subscription(std_msgs.msg.String, 'topic',
                                            lambda msg: msgs_rx.append(msg), 10)
        try:
            # Wait until the talker transmits two messages over the ROS topic.
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_rx) > 2:
                    break

            self.assertGreater(len(msgs_rx), 2)
        finally:
            self.node.destroy_subscription(sub)
