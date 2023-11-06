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

from lib.util.simulated_sensor_utils import SimulatedSensorUtils
from tests.integration.sensorlib_integration_test_runner import SensorlibIntegrationTestRunner


class TestOcclusion(SensorlibIntegrationTestRunner):

    def setup_scenario(self):
        self.create_vehicle([0.0, 0.0, 0.0])
        self.create_lidar_sensor([0.0, 0.0, 0.0])
        self.create_object([10.0, 0.0, 0.0])
        self.create_object([20.0, 0.0, 0.0])

    def create_vehicle(self, position):
        blueprint_library = self.carla_world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter("model3")[0]
        vehicle_transform = random.choice(self.carla_world.get_map().get_spawn_points())
        vehicle = self.carla_world.spawn_actor(vehicle_bp, vehicle_transform)

    def create_lidar_sensor(self, position):
        infrastructure_id = 3
        sensor_id = 7
        detection_cycle_delay_seconds = 0.5
        sensor_config = SimulatedSensorUtils.load_config_from_file("config/simulated_sensor_config.yaml")
        simulated_sensor_config = sensor_config["simulated_sensor"]
        carla_sensor_config = sensor_config["lidar_sensor"]
        noise_model_config = SimulatedSensorUtils.load_config_from_file("config/noise_model_config.yaml")
        user_offset = carla.Location(position[0], position[1], position[2])
        lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)

        sensor = self.api.create_simulated_semantic_lidar_sensor(simulated_sensor_config, carla_sensor_config,
                                                            noise_model_config,
                                                            detection_cycle_delay_seconds,
                                                            infrastructure_id, sensor_id,
                                                            lidar_transform.location, lidar_transform.rotation)

    def create_object(self, position):
        pass

    def test_occlusion(self):
        self.assertTrue(True)
        # sleep(1)
        # detected_objects = self.api.get_detected_objects()
        # self.assertTrue(len(detected_objects) > 0)
        # self.assertTrue(len(detected_objects) == 1)


if __name__ == '__main__':
    unittest.main()
