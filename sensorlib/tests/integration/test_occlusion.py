# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import os
import unittest
import random
from time import sleep

import carla

from sensorlib.util.simulated_sensor_utils import SimulatedSensorUtils
from tests.integration.integration_test_utilities import IntegrationTestUtilities
from tests.integration.sensorlib_integration_test_runner import SensorlibIntegrationTestRunner


class TestOcclusion(SensorlibIntegrationTestRunner):

    def test_non_occluded(self):

        # vehicle_position = self.carla_world.get_map().get_spawn_points()[1].location
        # print(vehicle_position)
        vehicle_position = carla.Location(65.516594, 7.808423, 0.275307)
        self.infrastructure_id = 3
        self.sensor_id = 7

        IntegrationTestUtilities.create_vehicle(self.carla_world, vehicle_position)
        IntegrationTestUtilities.create_lidar_sensor(self.api, 0, 0, vehicle_position)
        IntegrationTestUtilities.create_object(self.carla_world, vehicle_position + carla.Location(10.0, -4.0, 0.0))
        IntegrationTestUtilities.create_object(self.carla_world, vehicle_position + carla.Location(20.0, 0.0, 0.0))




    def test_occluded(self):
        self.assertTrue(True)
        sleep(1)
        detected_objects = self.api.get_detected_objects(self.infrastructure_id, self.sensor_id)
        print(detected_objects)
        self.assertTrue(len(detected_objects) > 0)
        # self.assertTrue(len(detected_objects) == 1)


if __name__ == '__main__':
    unittest.main()
