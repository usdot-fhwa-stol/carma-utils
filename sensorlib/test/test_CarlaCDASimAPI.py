# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
import sched
import time
import unittest
from unittest.mock import MagicMock

import carla
import numpy as np

sys.path.append('../')
from CarlaCDASimAPI import CarlaCDASimAPI
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestCarlaCDASimAPI(unittest.TestCase):
    def setUp(self):
        # Mock the CARLA objects
        self.carla_world = MagicMock()
        self.carla_client = MagicMock(get_world=MagicMock(return_value=self.carla_world))
        self.api = CarlaCDASimAPI.build_from_client(self.carla_client)

        # Configurations
        self.carla_sensor_config = SimulatedSensorTestUtils.generate_lidar_sensor_config()

    def test_build_from_host_spec(self):
        carla.Client = MagicMock(return_value=self.carla_client)
        api = CarlaCDASimAPI.build_from_host_spec("localhost", 2000)
        assert api._CarlaCDASimAPI__client == self.carla_client
        assert api._CarlaCDASimAPI__carla_world == self.carla_world
        assert api._CarlaCDASimAPI__infrastructure_sensors == {}

    def test_build_from_client(self):
        api = CarlaCDASimAPI.build_from_client(self.carla_client)
        assert api._CarlaCDASimAPI__client == self.carla_client
        assert api._CarlaCDASimAPI__carla_world == self.carla_world
        assert api._CarlaCDASimAPI__infrastructure_sensors == {}

    def test_build_from_world(self):
        api = CarlaCDASimAPI.build_from_world(self.carla_world)
        assert api._CarlaCDASimAPI__client == None
        assert api._CarlaCDASimAPI__carla_world == self.carla_world
        assert api._CarlaCDASimAPI__infrastructure_sensors == {}

    def test_create_simulated_semantic_lidar_sensor(self):
        # Values
        infrastructure_id = 3
        sensor_id = 7
        simulated_sensor_config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        carla_sensor_config = SimulatedSensorTestUtils.generate_lidar_sensor_config()
        noise_model_config = SimulatedSensorTestUtils.generate_noise_model_config()
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
        sensor = api.create_simulated_semantic_lidar_sensor(simulated_sensor_config, carla_sensor_config,
                                                            noise_model_config,
                                                            detection_cycle_delay_seconds,
                                                            infrastructure_id, sensor_id,
                                                            sensor_position, sensor_rotation, parent_actor_id)

        # Validate sensor fields have been correctly constructed
        assert sensor._infrastructure_id == infrastructure_id
        assert sensor._sensor_id == sensor_id
        assert sensor._SemanticLidarSensor__simulated_sensor_config == simulated_sensor_config
        assert sensor._SemanticLidarSensor__carla_sensor_config == carla_sensor_config
        assert sensor._SemanticLidarSensor__carla_world == carla_world
        assert sensor._SemanticLidarSensor__sensor is not None
        assert sensor._SemanticLidarSensor__data_collector is not None
        assert sensor._SemanticLidarSensor__noise_model is not None
        assert sensor._SemanticLidarSensor__detected_objects == []

        # Also validate retrieval through registration
        registered_sensor = api.get_simulated_sensor(infrastructure_id, sensor_id)
        assert sensor._infrastructure_id == registered_sensor._infrastructure_id

    def test_get_simulated_sensor(self):
        # Mock out registered sensors
        self.api._CarlaCDASimAPI__infrastructure_sensors = {(0, 0): MagicMock(id=0),
                                                       (0, 1): MagicMock(id=1),
                                                       (1, 0): MagicMock(id=2),
                                                       (1, 1): MagicMock(id=3)
                                                       }

        # Validate able to retrieve registered sensors
        assert self.api.get_simulated_sensor(0, 0).id == 0
        assert self.api.get_simulated_sensor(0, 1).id == 1
        assert self.api.get_simulated_sensor(1, 0).id == 2
        assert self.api.get_simulated_sensor(1, 1).id == 3

    def test_get_detected_objects(self):
        # Register a sensor
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        self.api._CarlaCDASimAPI__infrastructure_sensors = {
            (0, 0): MagicMock(id=0, get_detected_objects=MagicMock(return_value=detected_objects))}

        # Call
        retrieved_detected_objects = self.api.get_detected_objects(0, 0)

        # Validate
        assert retrieved_detected_objects == detected_objects

    def test_schedule_next_compute(self):
        scheduler = sched.scheduler(time.time, time.sleep)
        scheduler.enter = MagicMock(return_value=MagicMock())
        simulated_sensor = MagicMock()
        detection_cycle_delay_seconds = 0.1
        self.api._CarlaCDASimAPI__schedule_next_compute(scheduler, simulated_sensor, detection_cycle_delay_seconds)
        scheduler.enter.assert_called_with(detection_cycle_delay_seconds, 1, self.api._CarlaCDASimAPI__schedule_next_compute,
                                           (scheduler, simulated_sensor, detection_cycle_delay_seconds))

    def test_generate_lidar_bp(self):
        blueprint_library = MagicMock(find=MagicMock(return_value=MagicMock(set_attribute=MagicMock())))
        bp = self.api._CarlaCDASimAPI__generate_lidar_bp(blueprint_library, self.carla_sensor_config)
        bp.set_attribute.assert_called_with("points_per_second", "10000")
        assert isinstance(bp, MagicMock)
