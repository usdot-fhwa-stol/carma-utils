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

from sensorlib.util.carla_loader import CarlaLoader
import carla

from sensorlib.util.simulated_sensor_utils import SimulatedSensorUtils
from tests.integration.integration_test_utilities import IntegrationTestUtilities
from tests.integration.sensorlib_integration_test_runner import SensorlibIntegrationTestRunner


class TestOcclusion(SensorlibIntegrationTestRunner):

    def test_non_occluded(self):
        # Build vehicles
        vehicle_position = self.carla_world.get_map().get_spawn_points()[1].location
        # print(vehicle_position)
        # vehicle_position = carla.Location(65.516594, 7.808423, 0.275307)

        primary_vehicle = IntegrationTestUtilities.create_vehicle(self.carla_world, vehicle_position)
        middle_vehicle = IntegrationTestUtilities.create_object(self.carla_world,
                                                                vehicle_position + carla.Location(9.0, -4.0, 0.0))
        far_vehicle = IntegrationTestUtilities.create_object(self.carla_world,
                                                             vehicle_position + carla.Location(40.0, 0.0, 0.0))

        # Build sensor
        infrastructure_id = 0
        sensor_id = 0
        IntegrationTestUtilities.create_lidar_sensor(self.api,
                                                     infrastructure_id, sensor_id,
                                                     vehicle_position + carla.Location(0.0, 0.0, 0.5),
                                                     primary_vehicle.id)

        # Wait
        sleep(1)

        # Get detected objects removing the primary vehicle from consideration
        detected_objects = self.api.get_detected_objects(infrastructure_id, sensor_id)
        detected_objects = list(filter(lambda item: item.id != primary_vehicle.id, detected_objects))

        # Test number of detected objects
        print(f"Number of detected_objects: {len(detected_objects)}")
        for detected_object in detected_objects:
            print(detected_object)
        self.assertTrue(len(detected_objects) > 0)
        self.assertTrue(len(detected_objects) == 2)

    def test_occluded(self):
        # Build vehicles
        vehicle_position = self.carla_world.get_map().get_spawn_points()[1].location

        primary_vehicle = IntegrationTestUtilities.create_vehicle(self.carla_world, vehicle_position)
        middle_vehicle = IntegrationTestUtilities.create_object(self.carla_world,
                                                                vehicle_position + carla.Location(9.0, 0.0, 0.0))
        far_vehicle = IntegrationTestUtilities.create_object(self.carla_world,
                                                             vehicle_position + carla.Location(40.0, 0.0, 0.0))

        # Build sensor
        infrastructure_id = 0
        sensor_id = 0
        IntegrationTestUtilities.create_lidar_sensor(self.api,
                                                     infrastructure_id, sensor_id,
                                                     vehicle_position + carla.Location(0.0, 0.0, 0.5),
                                                     primary_vehicle.id)

        # Wait
        sleep(1)

        # Get detected objects removing the primary vehicle from consideration
        detected_objects = self.api.get_detected_objects(infrastructure_id, sensor_id)
        detected_objects = list(filter(lambda item: item.id != primary_vehicle.id, detected_objects))

        # Test number of detected objects
        print(f"Number of detected_objects: {len(detected_objects)}")
        for detected_object in detected_objects:
            print(detected_object)
        self.assertTrue(len(detected_objects) > 0)
        self.assertTrue(len(detected_objects) == 1)


if __name__ == '__main__':
    unittest.main()
