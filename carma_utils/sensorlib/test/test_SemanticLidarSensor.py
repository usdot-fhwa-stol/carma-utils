import unittest
from dataclasses import replace
from unittest.mock import MagicMock

import numpy as np

from src.SemanticLidarSensor import SemanticLidarSensor
from src.collector.SensorDataCollector import SensorDataCollector
from src.noise_models.GaussianNoiseModel import GaussianNoiseModel
from src.objects.CarlaSensor import CarlaSensorBuilder
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestSemanticLidarSensor(unittest.TestCase):

    def setUp(self):
        # Build the sensor
        self.raw_carla_sensor = SimulatedSensorTestUtils.generate_carla_sensor()
        self.carla_sensor = CarlaSensorBuilder.build_sensor(self.raw_carla_sensor)
        self.carla_sensor = replace(self.carla_sensor, position=np.array([1.0, 1.0, 0.0]))

        # Configs
        self.simulated_sensor_config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        self.carla_sensor_config = SimulatedSensorTestUtils.generate_lidar_sensor_config()
        self.noise_model_config = SimulatedSensorTestUtils.generate_noise_model_config()

        # Build a SemanticLidarSensor instance to access functions under test
        self.carla_world = MagicMock()
        self.data_collector = SensorDataCollector(self.carla_world, self.raw_carla_sensor)
        self.noise_model = GaussianNoiseModel(self.noise_model_config)
        self.sensor = SemanticLidarSensor(self.simulated_sensor_config, self.carla_sensor_config, self.carla_world,
                                          self.carla_sensor, self.data_collector, self.noise_model)

    def test_transform_to_sensor_frame(self):
        # Generate objects in original world frame
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        detected_objects = [
            replace(detected_objects[0], position=np.array([10.0, 10.0, 0.0])),
            replace(detected_objects[1], position=np.array([11.0, 9.0, 0.0])),
            replace(detected_objects[2], position=np.array([14.0, 9.0, 0.0])),
            replace(detected_objects[3], position=np.array([8.0, 10.0, 0.0])),
            replace(detected_objects[4], position=np.array([8.0, 14.0, 0.0])),
            replace(detected_objects[5], position=np.array([14.0, 14.0, 0.0]))
        ]

        # Execute
        new_detected_objects = self.sensor.transform_to_sensor_frame(detected_objects)

        # Assert the objects have been translated to the sensor-centric frame
        assert np.allclose(new_detected_objects[0].position, np.array([9.0, 9.0, 0.0]))
        assert np.allclose(new_detected_objects[1].position, np.array([10.0, 8.0, 0.0]))
        assert np.allclose(new_detected_objects[2].position, np.array([13.0, 8.0, 0.0]))
        assert np.allclose(new_detected_objects[3].position, np.array([7.0, 9.0, 0.0]))
        assert np.allclose(new_detected_objects[4].position, np.array([7.0, 13.0, 0.0]))
        assert np.allclose(new_detected_objects[5].position, np.array([13.0, 13.0, 0.0]))