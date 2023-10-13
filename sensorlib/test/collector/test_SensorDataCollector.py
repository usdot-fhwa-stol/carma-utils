# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import unittest
from collections import deque
from unittest.mock import MagicMock

import numpy as np

from src.collector.SensorDataCollector import SensorDataCollector


class TestSensorDataCollector(unittest.TestCase):

    def setUp(self):
        self.carla_world = MagicMock()
        self.carla_sensor = MagicMock()
        self.sensor_data_collector = SensorDataCollector(self.carla_world, self.carla_sensor)

    def test_get_carla_lidar_hitpoints(self):
        # Create a sample data collection
        data_collection = [1, 2, 3]
        self.sensor_data_collector._SensorDataCollector__data = deque([list(), data_collection])

        # Test the get_carla_lidar_hitpoints function
        result = self.sensor_data_collector.get_carla_lidar_hitpoints()
        self.assertEqual(result, (0, data_collection))

    def test__collect_sensor_data(self):
        # Call __collect_sensor_data
        raw_sensor_data = MagicMock(horizontal_angle=1.0, raw_data=[
            MagicMock(object_idx=0, point=MagicMock(x=0.0, y=0.0, z=0.0)),
            MagicMock(object_idx=1, point=MagicMock(x=1.0, y=1.0, z=1.0)),
            MagicMock(object_idx=2, point=MagicMock(x=2.0, y=2.0, z=2.0))
        ])
        self.sensor_data_collector._SensorDataCollector__collect_sensor_data(raw_sensor_data)

        # Check if a new data collection is created and the raw_sensor_data is added to it
        self.assertEqual(len(self.sensor_data_collector._SensorDataCollector__data), 2)
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][0], [0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][1], [1.0, 1.0, 1.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][2], [2.0, 2.0, 2.0]))
        self.assertEqual(self.sensor_data_collector._SensorDataCollector__data[1], {})  # Prior

        # Call again with a data collection event in the same collection cycle (sensor_rotation_angle is increasing)
        raw_sensor_data = MagicMock(horizontal_angle=2.0, raw_data=[
            MagicMock(object_idx=3, point=MagicMock(x=3.0, y=3.0, z=3.0)),
            MagicMock(object_idx=4, point=MagicMock(x=4.0, y=4.0, z=4.0)),
            MagicMock(object_idx=5, point=MagicMock(x=5.0, y=5.0, z=5.0))
        ])
        self.sensor_data_collector._SensorDataCollector__collect_sensor_data(raw_sensor_data)

        # Check that the data was appended to the collection
        self.assertEqual(len(self.sensor_data_collector._SensorDataCollector__data), 2)
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][0], [0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][1], [1.0, 1.0, 1.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][2], [2.0, 2.0, 2.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][3], [3.0, 3.0, 3.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][4], [4.0, 4.0, 4.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][5], [5.0, 5.0, 5.0]))
        self.assertEqual(self.sensor_data_collector._SensorDataCollector__data[1], {})

        # Call again with a data collection event in the next collection cycle (sensor_rotation_angle is decreasing)
        raw_sensor_data = MagicMock(horizontal_angle=0.0, raw_data=[
            MagicMock(object_idx=6, point=MagicMock(x=6.0, y=6.0, z=6.0)),
            MagicMock(object_idx=7, point=MagicMock(x=7.0, y=7.0, z=7.0)),
            MagicMock(object_idx=8, point=MagicMock(x=8.0, y=8.0, z=8.0))
        ])
        self.sensor_data_collector._SensorDataCollector__collect_sensor_data(raw_sensor_data)

        # Check that the new data was placed in a new collection
        self.assertEqual(len(self.sensor_data_collector._SensorDataCollector__data), 2)
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[1][0], [0.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[1][1], [1.0, 1.0, 1.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[1][2], [2.0, 2.0, 2.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[1][3], [3.0, 3.0, 3.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[1][4], [4.0, 4.0, 4.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[1][5], [5.0, 5.0, 5.0]))

        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][6], [6.0, 6.0, 6.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][7], [7.0, 7.0, 7.0]))
        self.assertTrue(np.allclose(self.sensor_data_collector._SensorDataCollector__data[0][8], [8.0, 8.0, 8.0]))

    def test__is_same_data_collection(self):
        # Call __is_same_data_collection with a sensor_rotation_angle greater than prev_angle
        result = self.sensor_data_collector._SensorDataCollector__is_same_data_collection(0.0)
        result = self.sensor_data_collector._SensorDataCollector__is_same_data_collection(1.0)
        self.assertTrue(result)

        # Call __is_same_data_collection with a sensor_rotation_angle equal to prev_angle
        result = self.sensor_data_collector._SensorDataCollector__is_same_data_collection(1.0)
        self.assertFalse(result)

        # Call __is_same_data_collection with a sensor_rotation_angle less than prev_angle
        result = self.sensor_data_collector._SensorDataCollector__is_same_data_collection(-1.0)
        self.assertFalse(result)
