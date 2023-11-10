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
import open3d as o3d

from sensorlib.util.carla_loader import CarlaLoader
import carla

from sensorlib.util.simulated_sensor_utils import SimulatedSensorUtils
from tests.integration.integration_test_utilities import IntegrationTestUtilities
from tests.integration.sensorlib_integration_test_runner import SensorlibIntegrationTestRunner


class TestOcclusion(SensorlibIntegrationTestRunner):

    def setup_occlusion_test(self, infrastructure_id, sensor_id,
                             primary_vehicle_offset, middle_object_offset, far_object_offset):


        # Orient the spectator
        primary_vehicle_position = self.carla_world.get_map().get_spawn_points()[1].location
        IntegrationTestUtilities.set_spectator_position(self.carla_world, primary_vehicle_position + carla.Location(10.0, 40.0, 20.0),
                                                        -10.0, 0.0, -90.0)

        # Build vehicles
        # primary_vehicle = IntegrationTestUtilities.create_vehicle(self.carla_world,
        #                                                           primary_vehicle_position + primary_vehicle_offset)
        middle_object = IntegrationTestUtilities.create_vehicle(self.carla_world,
                                                               primary_vehicle_position + middle_object_offset)

        far_object = IntegrationTestUtilities.create_pedestrian(self.carla_world,
                                                            primary_vehicle_position + far_object_offset)

        # Build sensor, including display callback
        sensor_position = primary_vehicle_position + carla.Location(0.0, 0.0, 0.5)
        point_list = o3d.geometry.PointCloud()
        sensor = IntegrationTestUtilities.create_lidar_sensor(self.api,
                                                              infrastructure_id, sensor_id,
                                                              sensor_position,
                                                              None,
                                                              # primary_vehicle.id)
                                                              lambda data: self.semantic_lidar_callback(data, point_list))

        # Start windows
        sensor_config = SimulatedSensorUtils.load_config_from_file("config/simulated_sensor_config.yaml")
        carla_sensor_config = sensor_config["lidar_sensor"]
        # self.launch_display_windows(sensor, sensor_position, carla_sensor_config, point_list)

        # Run one computation loop
        sleep(1)
        sensor.compute_detected_objects()

        return None, middle_object, far_object
        # return primary_vehicle, middle_object, far_object

    def test_non_occluded(self):

        infrastructure_id = 0
        sensor_id = 0

        primary_vehicle, middle_object, far_object = self.setup_occlusion_test(
            infrastructure_id, sensor_id,
            carla.Location(0.0, 0.0, 0.0),
            carla.Location(9.0, -4.0, 0.0),
            carla.Location(40.0, 0.0, 0.0))

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

        infrastructure_id = 0
        sensor_id = 0

        primary_vehicle, middle_object, far_object = self.setup_occlusion_test(
            infrastructure_id, sensor_id,
            carla.Location(0.0, 0.0, 0.0),
            carla.Location(9.0, 0.0, 0.0),
            carla.Location(40.0, 0.0, 0.0))

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
