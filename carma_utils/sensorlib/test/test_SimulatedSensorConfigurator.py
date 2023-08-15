import unittest
from unittest.mock import MagicMock

import carla

from src.SemanticLidarSensor import SemanticLidarSensor
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

    def test_build_simulated_sensor(self):
        mock_sensor = MagicMock()
        sensor = SimulatedSensorConfigurator.build_simulated_sensor(self.mock_carla_world, MagicMock(), MagicMock(), "../config/simulated_sensor_config.yaml", "../config/noise_model_config.yaml")
        assert isinstance(sensor, SemanticLidarSensor)

    def test_build_carla_semantic_lidar_sensor(self):
        carla_sensor = MagicMock(return_value=MagicMock())
        carla_world = MagicMock()
        carla_world.get_blueprint_library = MagicMock(find=MagicMock(return_value=MagicMock(set_attribute=MagicMock())))
        carla_world.spawn_actor = carla_sensor
        lidar_sensor_config = SimulatedSensorTestUtils.generate_lidar_sensor_config()
        sensor = SimulatedSensorConfigurator.build_carla_semantic_lidar_sensor(carla_world, MagicMock(), MagicMock(), lidar_sensor_config)
        assert isinstance(sensor, MagicMock)

    def test_generate_lidar_bp(self):
        blueprint_library = MagicMock(find=MagicMock(return_value=MagicMock(set_attribute=MagicMock())))
        bp = SimulatedSensorConfigurator.generate_lidar_bp(blueprint_library, self.carla_sensor_config)
        bp.set_attribute.assert_called_with("points_per_second", "10000")
        assert isinstance(bp, MagicMock)

    def test_build_simulated_lidar_sensor(self):
        sensor = SimulatedSensorConfigurator.build_simulated_lidar_sensor(self.mock_carla_world, self.mock_carla_sensor,
                                                                          self.simulated_sensor_config,
                                                                          self.carla_sensor_config,
                                                                          self.noise_model_config)
        assert isinstance(sensor, SemanticLidarSensor)
