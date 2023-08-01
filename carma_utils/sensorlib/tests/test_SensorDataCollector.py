import unittest
from collections import deque

from src.SensorDataCollector import SensorDataCollector


class TestSensorDataCollector(unittest.TestCase):

    def setUp(self):
        self.carla_world = None
        self.carla_sensor = None
        self.sensor_data_collector = SensorDataCollector(self.carla_world, self.carla_sensor)

    def test_get_carla_lidar_hitpoints(self):
        # Create a sample data collection
        data_collection = [1, 2, 3]
        self.sensor_data_collector._SensorDataCollector__data = deque([list(), data_collection])

        # Test the get_carla_lidar_hitpoints function
        result = self.sensor_data_collector.get_carla_lidar_hitpoints()
        self.assertEqual(result, data_collection)

    def test__collect_sensor_data(self):
        # Call __collect_sensor_data
        sensor_rotation_angle = 1.0
        raw_sensor_data_0 = [1, 2, 3]  # In practice this will be a list of carla.SensorData objects
        self.sensor_data_collector._SensorDataCollector__collect_sensor_data(sensor_rotation_angle, raw_sensor_data_0)

        # Check if a new data collection is created and the raw_sensor_data is added to it
        self.assertEqual(len(self.sensor_data_collector._SensorDataCollector__data), 2)
        self.assertEqual(self.sensor_data_collector._SensorDataCollector__data[0], raw_sensor_data_0)  # Current
        self.assertEqual(self.sensor_data_collector._SensorDataCollector__data[1], [])  # Prior

        # Call again with a data collection event in the same collection cycle (sensor_rotation_angle is increasing)
        sensor_rotation_angle = 2.0
        raw_sensor_data_1 = [4, 5, 6]
        self.sensor_data_collector._SensorDataCollector__collect_sensor_data(sensor_rotation_angle, raw_sensor_data_1)

        # Check that the data was appended to the collection
        self.assertEqual(self.sensor_data_collector._SensorDataCollector__data[0],
                         raw_sensor_data_0 + raw_sensor_data_1)
        self.assertEqual(self.sensor_data_collector._SensorDataCollector__data[1], [])

        # Call again with a data collection event in the next collection cycle (sensor_rotation_angle is decreasing)
        sensor_rotation_angle = 0.0
        raw_sensor_data_2 = [7, 8, 9]
        self.sensor_data_collector._SensorDataCollector__collect_sensor_data(sensor_rotation_angle, raw_sensor_data_2)
        self.assertEqual(self.sensor_data_collector._SensorDataCollector__data[0], raw_sensor_data_2)
        self.assertEqual(self.sensor_data_collector._SensorDataCollector__data[1],
                         raw_sensor_data_0 + raw_sensor_data_1)

    def test__is_same_data_collection(self):
        # Call __is_same_data_collection with a sensor_rotation_angle greater than prev_angle
        result = self.sensor_data_collector._SensorDataCollector__is_same_data_collection(0.0)
        result = self.sensor_data_collector._SensorDataCollector__is_same_data_collection(1.0)
        self.assertTrue(result)

        # Call __is_same_data_collection with a sensor_rotation_angle equal to prev_angle
        result = self.sensor_data_collector._SensorDataCollector__is_same_data_collection(1.0)
        self.assertTrue(result)

        # Call __is_same_data_collection with a sensor_rotation_angle less than prev_angle
        result = self.sensor_data_collector._SensorDataCollector__is_same_data_collection(-1.0)
        self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()
