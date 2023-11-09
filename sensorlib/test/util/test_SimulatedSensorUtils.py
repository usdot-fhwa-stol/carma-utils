# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import os
import unittest

import numpy as np
import yaml

from util.SimulatedSensorUtils import SimulatedSensorUtils
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestSimulatedSensorUtils(unittest.TestCase):

    def test_load_config_from_file(self):
        config_file_path = "test__simulated_sensor_config.yaml"
        config = SimulatedSensorTestUtils.generate_simulated_sensor_config()

        with open(config_file_path, "w") as file:
            yaml.dump(config, file)

        loaded_config = SimulatedSensorUtils.load_config_from_file(config_file_path)

        self.assertEqual(config, loaded_config)
        os.remove(config_file_path)

    def test_serialize_to_json(self):
        class TestClass:
            def __init__(self):
                self.a = "Some string."
                self.b = np.array([20.1, 30.0, 40.1])
                self.c = 3
                self.d = np.array([[1, 2, 3], [4, 5, 6]])

        test_obj = TestClass()

        # Test single-object serialization containing numpy arrays
        json_str = SimulatedSensorUtils.serialize_to_json(test_obj)
        expected_json_str = '{"a": "Some string.", "b": [20.1, 30.0, 40.1], "c": 3, "d": [[1, 2, 3], [4, 5, 6]]}'
        self.assertEqual(json_str, expected_json_str)

        # Test list of objects containing numpy arrays
        json_str_list = SimulatedSensorUtils.serialize_to_json([test_obj, test_obj])
        self.assertEqual(json_str_list,
                         '["{\\"a\\": \\"Some string.\\", \\"b\\": [20.1, 30.0, 40.1], \\"c\\": 3, \\"d\\": [[1, 2, 3], [4, 5, 6]]}", "{\\"a\\": \\"Some string.\\", \\"b\\": [20.1, 30.0, 40.1], \\"c\\": 3, \\"d\\": [[1, 2, 3], [4, 5, 6]]}"]')
