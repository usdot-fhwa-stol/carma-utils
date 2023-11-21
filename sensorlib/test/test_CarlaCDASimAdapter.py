# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
import json
import time
import unittest
import sys
from dataclasses import replace
from unittest.mock import MagicMock

import numpy as np
sys.path.append('../')


from src.CarlaCDASimAPI import CarlaCDASimAPI
from src.CarlaCDASimAdapter import CarlaCDASimAdapter
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestCarlaCDASimAdapter(unittest.TestCase):

    def setUp(self):

        # Mock the CARLA objects
        self.carla_world = MagicMock()
        self.carla_client = MagicMock(get_world=MagicMock(return_value=self.carla_world))
        self.api = CarlaCDASimAPI.build_from_client(self.carla_client)
        self.data_service = CarlaCDASimAdapter(self.api)

        # Configurations
        self.carla_sensor_config = SimulatedSensorTestUtils.generate_lidar_sensor_config()

    def test_start_xml_rpc_server(self):

        # Launch the server locally
        rpc_server_thread = self.data_service.start_xml_rpc_server("localhost", 2000, "../config/simulated_sensor_config.yaml", "../config/noise_model_config.yaml", 0.5, False)

        # Shut the server down
        time.sleep(2)
        try:
            rpc_server_thread._stop()
        except:
            pass

    def test_create_simulated_semantic_lidar_sensor(self):

        # Values
        infrastructure_id = 3
        sensor_id = 7
        detection_cycle_delay_seconds = 0.1
        sensor_position = np.array([1.0, 2.0, 3.0])
        sensor_rotation = np.array([0.0, 0.0, 0.0])
        parent_actor_id = 4

        # Mock the internal functions
        carla_world = MagicMock(
            get_blueprint_library=MagicMock(find=MagicMock(return_value=MagicMock(set_attribute=MagicMock()))),
            spawn_actor=MagicMock(return_value=MagicMock(id=infrastructure_id)))

        carla_sensor = SimulatedSensorTestUtils.generate_carla_sensor()
        carla_sensor.listen = MagicMock(return_value=MagicMock())

        # Build and register the sensor
        api = CarlaCDASimAPI.build_from_world(carla_world)
        data_service = CarlaCDASimAdapter(api)
        rpc_server_thread = self.data_service.start_xml_rpc_server("localhost", 2000, "../config/simulated_sensor_config.yaml", "../config/noise_model_config.yaml", 0.5, False)

        new_sensor_id = data_service._CarlaCDASimAdapter__create_simulated_semantic_lidar_sensor(
            infrastructure_id, sensor_id,
            sensor_position, sensor_rotation)

        # Validate sensor fields have been correctly constructed
        assert new_sensor_id == str(sensor_id)

        # Also validate retrieval through registration
        assert str(sensor_id) == data_service._CarlaCDASimAdapter__get_simulated_sensor(infrastructure_id, sensor_id)

    def test_get_simulated_sensor(self):

        # Mock out registered sensors
        self.data_service._CarlaCDASimAdapter__api._CarlaCDASimAPI__infrastructure_sensors = {
            (0, 0): MagicMock(get_id=MagicMock(return_value=0)),
            (0, 1): MagicMock(get_id=MagicMock(return_value=1)),
            (1, 0): MagicMock(get_id=MagicMock(return_value=2)),
            (1, 1): MagicMock(get_id=MagicMock(return_value=3))
        }

        # Validate able to retrieve registered sensors
        assert self.data_service._CarlaCDASimAdapter__get_simulated_sensor(0, 0) == "0"
        assert self.data_service._CarlaCDASimAdapter__get_simulated_sensor(0, 1) == "1"
        assert self.data_service._CarlaCDASimAdapter__get_simulated_sensor(1, 0) == "2"
        assert self.data_service._CarlaCDASimAdapter__get_simulated_sensor(1, 1) == "3"

    def test_get_detected_objects(self):
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        detected_objects = [replace(obj, carla_actor=None) for obj in detected_objects]
        sensor = MagicMock(get_detected_objects=MagicMock(return_value=detected_objects))
        self.data_service._CarlaCDASimAdapter__api._CarlaCDASimAPI__infrastructure_sensors = {(0, 0): sensor}
        serialized = self.data_service._CarlaCDASimAdapter__get_detected_objects(0, 0)

        with open("data/test_data_serialized_detected_objects.json", "r") as file:
            expected_serialized_data = json.load(file)
            assert serialized == expected_serialized_data
