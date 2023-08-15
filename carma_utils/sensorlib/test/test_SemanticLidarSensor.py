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

























































    def test_apply_occlusion(self):
        detected_object = MagicMock(id=1)
        actor_angular_extents = {1: (0.0, 1.096)}
        hitpoints = {1: []}
        detection_thresholds = {1: 0.5}

        # Mock internal function
        self.sensor.is_visible = MagicMock(return_value=detected_object)

        # Call and apply assertions
        self.sensor.apply_occlusion([detected_object], actor_angular_extents, hitpoints, detection_thresholds)

        self.sensor.is_visible.assert_called_with(actor_angular_extents[detected_object.id],
                                                  hitpoints[detected_object.id],
                                                  detection_thresholds[detected_object.id]
                                                  )


    def test_is_visible(self):
        carla_sensor = MagicMock(points_per_second=10000, rotation_frequency=10, fov_angular_width=1.096)
        self.sensor._SemanticLidarSensor__sensor = carla_sensor
        fov = 1.096
        detection_threshold_ratio = 0.5

        # Test case 1: Number of hitpoints is greater than or equal to the minimum required
        object_hitpoints = []
        for i in range(4000):
            object_hitpoints += [MagicMock()]
        result = self.sensor.is_visible((fov, fov), object_hitpoints, detection_threshold_ratio)
        self.assertTrue(result)

        # Test case 2: Number of hitpoints is less than the minimum required
        object_hitpoints = []
        for i in range(4):
            object_hitpoints += [MagicMock()]
        result = self.sensor.is_visible((fov, fov), object_hitpoints, detection_threshold_ratio)
        self.assertFalse(result)






    def test_compute_expected_num_hitpoints(self):
        carla_sensor = MagicMock(points_per_second=10000, rotation_frequency=10, fov_angular_width=1.096)
        self.sensor._SemanticLidarSensor__sensor = carla_sensor

        fov = 1.096
        num_points_per_scan = carla_sensor.points_per_second / carla_sensor.rotation_frequency
        theta_resolution = carla_sensor.fov_angular_width / num_points_per_scan
        expected_result = fov / theta_resolution

        result = self.sensor.compute_expected_num_hitpoints(fov)

        self.assertEqual(result, expected_result)








    def test_apply_noise(self):
        detected_objects = [MagicMock(), MagicMock()]
        noise_model = MagicMock()

        # Mock the noise_model functions
        noise_model.apply_position_noise = MagicMock(return_value=detected_objects)
        noise_model.apply_orientation_noise = MagicMock(return_value=detected_objects)
        noise_model.apply_type_noise = MagicMock(return_value=detected_objects)
        noise_model.apply_list_inclusion_noise = MagicMock(return_value=detected_objects)

        # Test the method
        self.sensor._SemanticLidarSensor__noise_model = noise_model
        self.sensor.apply_noise(detected_objects)

        # Verify the noise model functions were called
        noise_model.apply_position_noise.assert_called_once_with(detected_objects)
        noise_model.apply_orientation_noise.assert_called_once_with(detected_objects)
        noise_model.apply_type_noise.assert_called_once_with(detected_objects)
        noise_model.apply_list_inclusion_noise.assert_called_once_with(detected_objects)

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
