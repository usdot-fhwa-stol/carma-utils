import unittest
from unittest.mock import MagicMock

import carla
import numpy as np

from src.objects.CarlaSensor import CarlaSensor

class CarlaSensorTestCase(unittest.TestCase):

    def setUp(self):
        carla_sensor = MagicMock()
        self.proxy_sensor = CarlaSensor(carla_sensor)

    def test_get_position(self):
        carla_sensor = self.proxy_sensor._CarlaSensor__carla_sensor
        expected_position = carla.Location(1.0, 2.0, 3.0)
        carla_sensor.get_location.return_value = expected_position

        position = self.proxy_sensor.get_position()

        self.assertEqual(position, expected_position)
        carla_sensor.get_location.assert_called_once()

    def test_get_points_per_second(self):
        carla_sensor = self.proxy_sensor._CarlaSensor__carla_sensor
        expected_points_per_second = 1000
        carla_sensor.points_per_second = expected_points_per_second

        points_per_second = self.proxy_sensor.get_points_per_second()

        self.assertEqual(points_per_second, expected_points_per_second)

    def test_get_rotation_frequency(self):
        carla_sensor = self.proxy_sensor._CarlaSensor__carla_sensor
        expected_rotation_frequency = 700
        carla_sensor.rotation_frequency = expected_rotation_frequency

        rotation_frequency = self.proxy_sensor.get_rotation_frequency()

        self.assertEqual(rotation_frequency, expected_rotation_frequency)

    def test_get_fov_angular_width(self):
        carla_sensor = self.proxy_sensor._CarlaSensor__carla_sensor
        expected_upper_fov = 30
        expected_lower_fov = -10
        carla_sensor.upper_fov = expected_upper_fov
        carla_sensor.lower_fov = expected_lower_fov

        fov_angular_width = self.proxy_sensor.get_fov_angular_width()

        expected_angular_width = np.deg2rad(expected_upper_fov - expected_lower_fov)
        self.assertEqual(fov_angular_width, expected_angular_width)

if __name__ == '__main__':
    unittest.main()
