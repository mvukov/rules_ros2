# Copyright 2025 Arjuna Ariyaratne
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

import rclpy
from rclpy.node import Node

from ros2.test.generate_parameter.test_parameters import test_params


class TestGenerateParameters(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_string_load(self):
        node = Node('test_node')
        param_listener = test_params.ParamListener(node)
        params = param_listener.get_params()

        self.assertEqual(params.string_test, 'default_string')
        node.destroy_node()

    def test_double_load(self):
        node = Node('test_node')
        param_listener = test_params.ParamListener(node)
        params = param_listener.get_params()

        self.assertAlmostEqual(params.double_test, 2.0)
        node.destroy_node()

    def test_int_load(self):
        node = Node('test_node')
        param_listener = test_params.ParamListener(node)
        params = param_listener.get_params()

        self.assertEqual(params.int_test, 5)
        node.destroy_node()

    def test_bool_load(self):
        node = Node('test_node')
        param_listener = test_params.ParamListener(node)
        params = param_listener.get_params()

        self.assertEqual(params.bool_test, True)
        node.destroy_node()

    def test_string_array_load(self):
        node = Node('test_node')
        param_listener = test_params.ParamListener(node)
        params = param_listener.get_params()

        expected = ['string_1', 'string_2']
        self.assertEqual(params.string_array_test, expected)
        node.destroy_node()

    def test_double_array_load(self):
        node = Node('test_node')
        param_listener = test_params.ParamListener(node)
        params = param_listener.get_params()

        expected = [1.0, 2.0, 3.0]
        self.assertEqual(params.double_array_test, expected)
        node.destroy_node()

    def test_int_array_load(self):
        node = Node('test_node')
        param_listener = test_params.ParamListener(node)
        params = param_listener.get_params()

        expected = [1, 2, 3, 4]
        self.assertEqual(params.int_array_test, expected)
        node.destroy_node()

    def test_bool_array_load(self):
        node = Node('test_node')
        param_listener = test_params.ParamListener(node)
        params = param_listener.get_params()

        expected = [True, False, True]
        self.assertEqual(params.bool_array_test, expected)
        node.destroy_node()

    def test_string_fixed_10_load(self):
        node = Node('test_node')
        param_listener = test_params.ParamListener(node)
        params = param_listener.get_params()

        self.assertEqual(params.string_fixed_10_test, 'fixed')
        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
