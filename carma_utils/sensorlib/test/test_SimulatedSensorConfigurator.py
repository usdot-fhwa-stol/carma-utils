# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import unittest
from unittest.mock import MagicMock

import carla

from src.SimulatedSensorConfigurator import SimulatedSensorConfigurator
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestSimulatedSensorConfigurator(unittest.TestCase):
    def setUp(self):
        # Mock the CARLA objects
        self.mock_carla_world = MagicMock()

        self.mock_carla_sensor = MagicMock()
        self.mock_carla_sensor.get_location = MagicMock(return_value=carla.Location(1.0, 2.0, 3.0))
        self.mock_carla_sensor.points_per_second = 1000
        self.mock_carla_sensor.rotation_frequency = 700
        self.mock_carla_sensor.upper_fov = 30
        self.mock_carla_sensor.lower_fov = -10

        # Configurations
        self.simulated_sensor_config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        self.carla_sensor_config = SimulatedSensorTestUtils.generate_lidar_sensor_config()
        self.noise_model_config = SimulatedSensorTestUtils.generate_noise_model_config()

    def test_register_simulated_semantic_lidar_sensor(self):
        # Values
        infrastructure_id = 3
        simulated_sensor_config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        carla_sensor_config = SimulatedSensorTestUtils.generate_lidar_sensor_config()
        noise_model_config = SimulatedSensorTestUtils.generate_noise_model_config()
        sensor_transform = carla.Transform(carla.Location(1.0, 2.0, 3.0), carla.Rotation(0.0, 0.0, 0.0))
        parent_actor = MagicMock()

        # Mock the internal functions
        carla_world = MagicMock(
            get_blueprint_library=MagicMock(find=MagicMock(return_value=MagicMock(set_attribute=MagicMock()))),
            spawn_actor=MagicMock(return_value=MagicMock(id=infrastructure_id)))

        carla_sensor = SimulatedSensorTestUtils.generate_carla_sensor()
        carla_sensor.listen = MagicMock(return_value=MagicMock())

        SimulatedSensorConfigurator._SimulatedSensorConfigurator__get_initialized_carla_world = MagicMock(
            return_value=carla_world)

        # Build and register the sensor
        sensor = SimulatedSensorConfigurator.register_simulated_semantic_lidar_sensor(simulated_sensor_config,
                                                                                      carla_sensor_config,
                                                                                      noise_model_config,
                                                                                      infrastructure_id,
                                                                                      sensor_transform, parent_actor)

        # Validate sensor fields have been correctly constructed
        assert sensor.infrastructure_id == infrastructure_id
        assert sensor._SemanticLidarSensor__simulated_sensor_config == simulated_sensor_config
        assert sensor._SemanticLidarSensor__carla_sensor_config == carla_sensor_config
        assert sensor._SemanticLidarSensor__carla_world == carla_world
        assert sensor._SemanticLidarSensor__sensor is not None
        assert sensor._SemanticLidarSensor__data_collector is not None
        assert sensor._SemanticLidarSensor__noise_model is not None
        assert sensor._SemanticLidarSensor__detected_objects == []

        # Also validate retrieval through registration
        registered_sensor = SimulatedSensorConfigurator.get_simulated_sensor(infrastructure_id)
        assert sensor.infrastructure_id == registered_sensor.infrastructure_id

    def test_get_simulated_sensor(self):
        # Assert empty registry upon initialization
        assert SimulatedSensorConfigurator.get_simulated_sensor(0) is None

        # Mock out registered sensors
        SimulatedSensorConfigurator._SimulatedSensorConfigurator__infrastructure_sensors = {0: MagicMock(id=0),
                                                                                            1: MagicMock(id=1)}

        # Validate able to retrieve registered sensors
        sensor0 = SimulatedSensorConfigurator.get_simulated_sensor(0)
        assert sensor0.id == 0
        sensor1 = SimulatedSensorConfigurator.get_simulated_sensor(1)
        assert sensor1.id == 1

    def test_get_initialized_carla_world(self):
        carla.Client = MagicMock(return_value=MagicMock(set_timeout=MagicMock(return_value=MagicMock()),
                                                        get_world=MagicMock(return_value=MagicMock(id=3))))
        world = SimulatedSensorConfigurator._SimulatedSensorConfigurator__get_initialized_carla_world(
            self.simulated_sensor_config)
        assert world is not None
        assert world.id == 3

    def test_generate_lidar_bp(self):
        blueprint_library = MagicMock(find=MagicMock(return_value=MagicMock(set_attribute=MagicMock())))
        bp = SimulatedSensorConfigurator._SimulatedSensorConfigurator__generate_lidar_bp(blueprint_library,
                                                                                         self.carla_sensor_config)
        bp.set_attribute.assert_called_with("points_per_second", "10000")
        assert isinstance(bp, MagicMock)
