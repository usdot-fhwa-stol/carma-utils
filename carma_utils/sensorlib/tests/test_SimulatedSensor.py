import os
import unittest
from unittest.mock import MagicMock, patch

import carla
import numpy as np
import yaml

from src.SimulatedSensor import SimulatedSensor, SimulatedSensorUtils
from src.DetectedObject import DetectedObject
from src.SensorDataCollector import SensorDataCollector
from src.objects.ProxySensor import ProxySensor
from tests.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestSimulatedSensor(unittest.TestCase):

    def setUp(self):
        self.config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        self.carla_world = MagicMock()
        self.carla_sensor = SimulatedSensorTestUtils.generate_carla_sensor()
        self.sensor = ProxySensor(self.carla_sensor)
        self.noise_model = MagicMock()
        self.simulated_sensor = SimulatedSensor(self.config, self.carla_world, self.carla_sensor, self.noise_model)
        # Generate tests data
        self.detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_object_list(10)
        self.carla_lidar_hitpoints = SimulatedSensorTestUtils.generate_test_data_carla_lidar_hitpoints()

    def test_load_config_from_dict(self):
        config = SimulatedSensorTestUtils.generate_simulated_sensor_config()

        self.simulated_sensor.load_config_from_dict(config)

        self.assertEqual(self.simulated_sensor._SimulatedSensor__config, config)

    def test_load_config_from_file(self):
        config_file_path = "test__simulated_sensor_config.yaml"
        config = SimulatedSensorTestUtils.generate_simulated_sensor_config()

        with open(config_file_path, 'w') as file:
            yaml.dump(config, file)

        self.simulated_sensor.load_config_from_file(config_file_path)

        self.assertEqual(self.simulated_sensor._SimulatedSensor__config, config)
        os.remove(config_file_path)

    def test_get_detected_objects_in_frame__nominal(self):
        # Mock necessary objects and methods
        SimulatedSensorUtils.get_scene_detected_objects = MagicMock(return_value=self.detected_objects)
        SimulatedSensorUtils.prefilter = MagicMock(return_value=self.detected_objects)
        self.simulated_sensor._SimulatedSensor__raw_sensor_data_collector.get_carla_lidar_hitpoints.return_value = MagicMock(
            return_value=self.carla_lidar_hitpoints)
        SimulatedSensorUtils.compute_actor_angular_extents = MagicMock(return_value=self.detected_objects)
        SimulatedSensorUtils.compute_adjusted_detection_thresholds = MagicMock(return_value=self.detected_objects)
        SimulatedSensorUtils.apply_occlusion = MagicMock(return_value=self.detected_objects)
        SimulatedSensorUtils.apply_noise = MagicMock(return_value=self.detected_objects)

        # Test the method
        result = self.simulated_sensor.get_detected_objects_in_frame()

        # Assertions
        self.asserEqual(result, self.detected_objects)
        SimulatedSensorUtils.get_sensor.assert_called_once()
        SimulatedSensorUtils.get_scene_detected_objects.assert_called_once()
        SimulatedSensorUtils.prefilter.assert_called_once()
        self.simulated_sensor._SimulatedSensor__raw_sensor_data_collector.get_carla_lidar_hitpoints.assert_called_once()
        SimulatedSensorUtils.compute_actor_angular_extents.assert_called_once()
        SimulatedSensorUtils.compute_adjusted_detection_thresholds.assert_called_once()
        SimulatedSensorUtils.apply_occlusion.assert_called_once()
        SimulatedSensorUtils.apply_noise.assert_called_once()


class TestSimulatedSensorUtilities(unittest.TestCase):

    def setUp(self):
        self.config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        self.carla_world = MagicMock()
        self.carla_sensor = SimulatedSensorTestUtils.generate_carla_sensor()
        self.sensor = ProxySensor(self.carla_sensor)
        self.noise_model = MagicMock()
        self.simulated_sensor = SimulatedSensor(self.config, self.carla_world, self.carla_sensor, self.noise_model)
        # Generate tests data
        self.detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_object_list(10)
        self.carla_lidar_hitpoints = SimulatedSensorTestUtils.generate_test_data_carla_lidar_hitpoints()

    def test_get_scene_detected_objects(self):

        self.carla_world.get_actors = MagicMock(return_value=self.detected_objects)
        result = SimulatedSensorUtils.get_scene_detected_objects(self.carla_world, self.config)
        self.assertEqual(len(result), len(self.detected_objects))
        for i in range(len(result)):
            self.assertEqual(result[i].id, self.detected_objects[i].id)

    def test_prefilter(self):

        # Test filtering by type
        detected_objects = self.detected_objects[0:3]
        detected_objects[2].object_type = "Pole"
        result = SimulatedSensorUtils.prefilter(self.sensor, detected_objects, self.config)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0].object_type, "Pedestrian")
        self.assertEqual(result[1].object_type, "Vehicle")

        # Test filtering by distance (set in the configuration)
        result = SimulatedSensorUtils.prefilter(self.sensor, self.detected_objects, self.config)

        self.assertEqual(len(result), 4)

    def test_compute_actor_angular_extents(self):
        detected_objects = self.detected_objects[0:1]
        sensor = self.sensor
        sensor.get_position = MagicMock(return_value=np.array([0.0, 0.0, 0.0]))

        result = SimulatedSensorUtils.compute_actor_angular_extents(sensor, detected_objects)

        self.assertEqual(len(result), 1)
        self.assertEqual(result[0], (1.00574598146173, 1.09083078249646))

    def test_compute_view_angle(self):
        sensor = self.sensor
        self.sensor.position = np.array([0.0, 0.0, 0.0])
        result = SimulatedSensorUtils.compute_view_angle(sensor, np.array([1.0, 1.0, 0.0]))
        self.assertTrue(result, np.pi / 4)

    def test_compute_adjusted_detection_thresholds(self):
        sensor = self.sensor
        sensor.get_position = MagicMock(return_value=np.array([0.0, 0.0, 0.0]))
        detected_objects = [MagicMock(id=1)]
        relative_object_position_vectors = [np.array([3.0, 4.0, 0.0])]
        result = SimulatedSensorUtils.compute_adjusted_detection_thresholds(config, sensor, detected_objects)

        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0], 5.0)

    def test_compute_range(self):
        relative_object_position_vector = np.array([3.0, 4.0, 0.0])
        result = SimulatedSensorUtils.compute_range(relative_object_position_vector)
        self.assertAlmostEqual(result, 5.0)

    def test_apply_occlusion(self):
        detected_objects = [MagicMock(id=1), MagicMock(id=2)]
        actor_angular_extents = {1: (0.0, 1.096), 2: (2.0, 2.2)}
        sensor = MagicMock()
        hitpoints = {1: [MagicMock(), MagicMock()], 2: [MagicMock(), MagicMock()]}
        detection_thresholds = {1: 0.5, 2: 0.5}

        result = SimulatedSensorUtils.apply_occlusion(detected_objects, actor_angular_extents, sensor, hitpoints,
                                                      detection_thresholds)

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0], detected_objects[0])
        self.assertEqual(result[1], detected_objects[1])

    def test_is_visible(self):
        detected_object = MagicMock()
        actor_angular_extent = (0.0, 1.096)
        sensor = MagicMock()

        # Test case 1: Number of hitpoints is greater than or equal to the minimum required
        hitpoints = []
        for i in range(10000):
            hitpoints[1] += [MagicMock()]
        detection_thresholds = {1: 0.5}
        result = SimulatedSensorUtils.is_visible(detected_object, actor_angular_extent, sensor, hitpoints,
                                                 detection_thresholds)
        self.assertTrue(result)

        # Test case 2: Number of hitpoints is less than the minimum required
        hitpoints = []
        for i in range(4):
            hitpoints[1] += [MagicMock()]
        result = SimulatedSensorUtils.is_visible(detected_object, actor_angular_extent, sensor, hitpoints,
                                                 detection_thresholds)
        self.assertFalse(result)

    def test_compute_expected_num_hitpoints(self):
        sensor = MagicMock(points_per_second=10000, rotation_frequency=10, fov_angular_width=1.096)

        actor_angular_extent = (0.0, 1.096)
        num_points_per_scan = sensor.points_per_second / sensor.rotation_frequency
        theta_resolution = sensor.fov_angular_width / num_points_per_scan
        expected_result = (actor_angular_extent[1] - actor_angular_extent[0]) / theta_resolution

        result = SimulatedSensorUtils.compute_expected_num_hitpoints(actor_angular_extent, sensor)

        self.assertEqual(result, expected_result)

    def test_apply_noise(self):
        detected_objects = [MagicMock(), MagicMock()]
        noise_model = MagicMock()

        # Mock the noise_model.apply_position_noise method
        noise_model.apply_position_noise = MagicMock()
        noise_model.apply_position_noise.return_value = None

        # Mock the noise_model.apply_orientation_noise method
        noise_model.apply_orientation_noise = MagicMock()
        noise_model.apply_orientation_noise.return_value = None

        # Mock the noise_model.apply_type_noise method
        noise_model.apply_type_noise = MagicMock()
        noise_model.apply_type_noise.return_value = None

        # Mock the noise_model.apply_list_inclusion_noise method
        noise_model.apply_list_inclusion_noise = MagicMock()
        noise_model.apply_list_inclusion_noise.return_value = None

        # Test the method
        SimulatedSensorUtils.apply_noise(detected_objects, noise_model)

        # Assertions
        noise_model.apply_position_noise.assert_called_once_with(detected_objects)
        noise_model.apply_orientation_noise.assert_called_once_with(detected_objects)
        noise_model.apply_type_noise.assert_called_once_with(detected_objects)
        noise_model.apply_list_inclusion_noise.assert_called_once_with(detected_objects)


if __name__ == '__main__':
    unittest.main()
