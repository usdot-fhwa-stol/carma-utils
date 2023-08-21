# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import unittest
from unittest.mock import MagicMock

import carla
import numpy as np

from src.objects.CarlaSensor import CarlaSensor, CarlaSensorBuilder


class TestCarlaSensor(unittest.TestCase):

    def setUp(self):
        self.carla_sensor = MagicMock()
        self.carla_sensor.get_location = MagicMock(return_value=carla.Location(1.0, 2.0, 3.0))
        self.carla_sensor.points_per_second = 1000
        self.carla_sensor.rotation_frequency = 700
        self.carla_sensor.upper_fov = 30
        self.carla_sensor.lower_fov = -10

    def test_builder(self):
        sensor = CarlaSensorBuilder.build_sensor(self.carla_sensor)

        assert sensor.carla_sensor == self.carla_sensor
        assert (sensor.position == np.array([1.0, 2.0, 3.0])).all()
        assert sensor.points_per_second == 1000
        assert sensor.rotation_frequency == 700
        assert sensor.fov_angular_width == np.deg2rad(40)


if __name__ == "__main__":
    unittest.main()
