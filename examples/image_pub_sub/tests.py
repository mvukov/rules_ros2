#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 Yuki Furuta
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
import launch_testing.markers
import rclpy
import sensor_msgs.msg


@launch_testing.markers.keep_alive
def generate_test_description():
    publisher_node = launch_ros.actions.Node(
        name='image_publisher',
        executable='image_pub_sub/image_publisher',
        parameters=[
            {
                'width': 640,
                'height': 480,
                'frequency': 10.0,
            },
        ],
    )

    resizer_node = launch_ros.actions.Node(
        name='image_resizer',
        executable='image_pub_sub/image_resizer',
        parameters=[
            {
                'scale': 0.5,
                'input.image_transport': 'rotated',
            },
        ],
        remappings=[
            ('~/input/rotated', 'image_publisher/output/rotated'),
        ],
    )

    return (
        launch.LaunchDescription([
            publisher_node,
            resizer_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'image_publisher': publisher_node,
            'image_resizer': resizer_node,
        })


class TestImagePubSub(unittest.TestCase):
    """Tests image_transport functionality with rotated plugin.

    This test verifies that:
    1. The image_publisher node publishes images using rotated transport
    2. The image_resizer node receives images via rotated transport and publishes resized versions
    3. The rotated image transport plugin correctly rotates and restores images
    4. Image transport works correctly with custom transport plugins
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('image_pub_sub_test')

    def tearDown(self):
        self.node.destroy_node()

    def test_topic_advertised(self, launch_service, image_publisher, image_resizer, proc_output):
        end_time = time.time() + 10.0
        topic_info = []
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            topic_info = self.node.get_topic_names_and_types()
            if topic_info:
                break
        advertised_topic_names = {name for name, type in topic_info}
        required_topic_names = {
            '/image_publisher/output',
            '/image_publisher/output/rotated',
            '/image_resizer/output',
            '/image_resizer/output/rotated',
        }
        self.assertTrue(required_topic_names.issubset(advertised_topic_names),
                        f'Required topics not advertised. Missing: {required_topic_names - advertised_topic_names}')


    def test_publisher_transmits(self, launch_service, image_publisher, proc_output):
        """Test that the image_publisher publishes images on the expected topic."""
        msgs_rx = []

        sub = self.node.create_subscription(
            sensor_msgs.msg.Image,
            'image_publisher/output/rotated',
            lambda msg: msgs_rx.append(msg),
            10)

        try:
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_rx) > 2:
                    break

            self.assertGreater(len(msgs_rx), 2,
                               'image_publisher should have transmitted multiple images')
            # Verify image properties (width and height are swapped due to +90 deg rotation).
            self.assertEqual(msgs_rx[0].width, 480)
            self.assertEqual(msgs_rx[0].height, 640)
            self.assertEqual(msgs_rx[0].encoding, 'bgr8')
        finally:
            self.node.destroy_subscription(sub)

    def test_resizer_transmits(self, launch_service, image_resizer, proc_output):
        """Test that the image_resizer receives and publishes resized images."""
        msgs_rx = []

        sub = self.node.create_subscription(
            sensor_msgs.msg.Image,
            'image_resizer/output',
            lambda msg: msgs_rx.append(msg),
            10)

        try:
            end_time = time.time() + 10
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_rx) > 2:
                    break

            self.assertGreater(len(msgs_rx), 2,
                               'image_resizer should have transmitted multiple images')
            # Verify resized image properties (should be 50% of original 640x480).
            # After restoration from rotated transport, dimensions are back to original ratio.
            self.assertEqual(msgs_rx[0].width, 320)
            self.assertEqual(msgs_rx[0].height, 240)
            self.assertEqual(msgs_rx[0].encoding, 'bgr8')
        finally:
            self.node.destroy_subscription(sub)
