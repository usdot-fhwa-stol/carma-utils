import os
import unittest
from unittest.mock import MagicMock
import numpy as np
import yaml

from src.SimulatedSensor import SimulatedSensor, SimulatedSensorUtilities
from src.SensedObject import SensedObject
from src.SensorDataCollector import SensorDataCollector
from test.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestSimulatedSensor(unittest.TestCase):
    # TODO Not reviewed

    def setUp(self):
        self.config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        self.carla_world = MagicMock()
        self.carla_sensor = MagicMock()
        self.noise_model = MagicMock()
        self.simulated_sensor = SimulatedSensor(self.config, self.carla_world, self.carla_sensor, self.noise_model)

    def test_load_config_from_dict(self):
        config =

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


def test_get_sensed_objects_in_frame__nominal(self):
    # Mocking necessary objects and methods
    carla_lidar_hitpoints = MagicMock()
    self.simulated_sensor._SimulatedSensor__raw_sensor_data_collector.get_carla_lidar_hitpoints.return_value = carla_lidar_hitpoints

    sensed_objects = SimulatedSensorTestUtils.generate_sensed_objects()
    SimulatedSensorUtilities.get_scene_sensed_objects = MagicMock(return_value=sensed_objects)

    # Test the method
    result = self.simulated_sensor.get_sensed_objects_in_frame()

    # Assertions
    self.assertEqual(result, sensed_objects)
    SimulatedSensorUtilities.get_scene_sensed_objects.assert_called_once()
    self.simulated_sensor._SimulatedSensor__raw_sensor_data_collector.get_carla_lidar_hitpoints.assert_called_once()

class SimulatedSensorUtilitiesTest(unittest.TestCase):

    def setUp(self):
        self.carla_sensor = MagicMock()

    def test_get_sensor(self):
        carla_sensor = {
            "channels": 4,
            "points_per_second": 10000,
            "rotation_frequency": 10,
            "upper_fov": 30,
            "lower_fov": -30,
            "sensor_tick": 0.1
        }
        self.carla_sensor.get_location.return_value = MagicMock(x=0.0, y=0.0, z=0.0)
        self.carla_sensor.get_transform.return_value = MagicMock(
            get_matrix=MagicMock(return_value=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]))

        result = SimulatedSensorUtilities.get_sensor(self.carla_sensor)

        self.assertEqual(result["channels"], carla_sensor["channels"])
        self.assertEqual(result["points_per_second"], carla_sensor["points_per_second"])
        self.assertEqual(result["rotation_frequency"], carla_sensor["rotation_frequency"])
        self.assertEqual(result["upper_fov"], carla_sensor["upper_fov"])
        self.assertEqual(result["lower_fov"], carla_sensor["lower_fov"])
        self.assertEqual(result["sensor_tick"], carla_sensor["sensor_tick"])
        self.assertTrue(np.array_equal(result["position"], np.array([0.0, 0.0, 0.0])))
        self.assertTrue(
            np.array_equal(result["rotation"], np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])))

    def test_prefilter(self):
        sensor = {
            "position": np.array([0.0, 0.0, 0.0])
        }
        sensed_objects = [SensedObject({}, MagicMock(object_type="Pedestrian", position=np.array([1.0, 0.0, 0.0]))),
                          SensedObject({}, MagicMock(object_type="Vehicle", position=np.array([2.0, 0.0, 0.0]))),
                          SensedObject({}, MagicMock(object_type="Cyclist", position=np.array([3.0, 0.0, 0.0])))]
        config = SimulatedSensorTestUtils.generate_simulated_sensor_config()

        result = SimulatedSensorUtilities.prefilter(sensor, sensed_objects, config)

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0].object_type, "Pedestrian")
        self.assertEqual(result[1].object_type, "Vehicle")

    def test_compute_actor_angular_extents(self):
        sensor = {
            "position": np.array([0.0, 0.0, 0.0])
        }
        sensed_objects = [SensedObject({}, MagicMock(id=1, bbox=MagicMock(extent=np.array([1.0, 1.0, 1.0])))),
                          SensedObject({}, MagicMock(id=2, bbox=MagicMock(extent=np.array([2.0, 2.0, 2.0]))))]

        result = SimulatedSensorUtilities.compute_actor_angular_extents(sensor, sensed_objects)

        self.assertEqual(len(result), 2)
        self.assertEqual(result[1], (0.0, 0.0))
        self.assertEqual(result[2], (0.0, 0.0))

    def test_compute_view_angle(self):
        relative_object_position_vectors = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

        result = SimulatedSensorUtilities.compute_view_angle(relative_object_position_vectors)

        self.assertTrue(np.array_equal(result, np.array([0.0, 0.0])))

    def test_compute_adjusted_detection_thresholds(self):
        sensed_objects = [MagicMock(id=1), MagicMock(id=2)]
        relative_object_position_vectors = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

        config = {
            "detection_threshold_scaling_formula": {
                "hitpoint_detection_ratio_threshold_per_meter_change_rate": 0.1,
                "nominal_hitpoint_detection_ratio_threshold": 0.5
            }
        }

        result = SimulatedSensorUtilities.compute_adjusted_detection_thresholds(sensed_objects,
                                                                                relative_object_position_vectors,
                                                                                config)

        self.assertEqual(len(result), 2)
        self.assertAlmostEqual(result[1], 0.05)
        self.assertAlmostEqual(result[2], 0.0)

    def test_compute_range(self):
        relative_object_position_vector = np.array([3.0, 4.0, 0.0])

        result = SimulatedSensorUtilities.compute_range(relative_object_position_vector)

        self.assertAlmostEqual(result, 5.0)

    def test_apply_occlusion(self):
        sensed_objects = [MagicMock(id=1), MagicMock(id=2)]
        actor_angular_extents = {1: (0.0, 0.0), 2: (0.0, 0.0)}
        sensor = MagicMock()
        hitpoints = {1: [MagicMock(), MagicMock()], 2: [MagicMock(), MagicMock()]}
        detection_thresholds = {1: 0.5, 2: 0.5}

        result = SimulatedSensorUtilities.apply_occlusion(sensed_objects, actor_angular_extents, sensor, hitpoints,
                                                          detection_thresholds)

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0], sensed_objects[0])
        self.assertEqual(result[1], sensed_objects[1])

    def test_is_visible(self):
        sensed_object = MagicMock()
        actor_angular_extent = (0.0, 0.0)
        sensor = MagicMock()
        hitpoints = {1: [MagicMock(), MagicMock(), MagicMock()]}
        detection_thresholds = {1: 0.5}

        # Test case 1: Number of hitpoints is greater than or equal to the minimum required
        hitpoints_count = len(hitpoints[1])
        detection_threshold = detection_thresholds[1]
        expected_result = hitpoints_count >= detection_threshold * len(hitpoints[1])

        result = SimulatedSensorUtilities.is_visible(sensed_object, actor_angular_extent, sensor, hitpoints,
                                                     detection_thresholds)

        self.assertEqual(result, expected_result)

        # Test case 2: Number of hitpoints is less than the minimum required
        hitpoints_count = len(hitpoints[1]) - 1
        expected_result = hitpoints_count >= detection_threshold * len(hitpoints[1])

        result = SimulatedSensorUtilities.is_visible(sensed_object, actor_angular_extent, sensor, hitpoints,
                                                     detection_thresholds)

        self.assertEqual(result, expected_result)

    def test_compute_expected_num_hitpoints(self):
        actor_angular_extent = (0.0, 0.0)
        sensor = MagicMock(points_per_second=10000, rotation_frequency=10, fov_angular_width=30)

        # Test case 1: actor angular extent is 0, expected number of hitpoints should be 0
        expected_result = 0

        result = SimulatedSensorUtilities.compute_expected_num_hitpoints(actor_angular_extent, sensor)

        self.assertEqual(result, expected_result)

        # Test case 2: actor angular extent is non-zero, expected number of hitpoints should be calculated
        actor_angular_extent = (0.0, 30.0)
        num_points_per_scan = sensor.points_per_second / sensor.rotation_frequency
        theta_resolution = sensor.fov_angular_width / num_points_per_scan
        expected_result = (actor_angular_extent[1] - actor_angular_extent[0]) / theta_resolution

        result = SimulatedSensorUtilities.compute_expected_num_hitpoints(actor_angular_extent, sensor)

        self.assertEqual(result, expected_result)

    def test_apply_noise(self):
        sensed_objects = [MagicMock(), MagicMock()]
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
        SimulatedSensorUtilities.apply_noise(sensed_objects, noise_model)

        # Assertions
        noise_model.apply_position_noise.assert_called_once_with(sensed_objects)
        noise_model.apply_orientation_noise.assert_called_once_with(sensed_objects)
        noise_model.apply_type_noise.assert_called_once_with(sensed_objects)
        noise_model.apply_list_inclusion_noise.assert_called_once_with(sensed_objects)





if __name__ == '__main__':
    unittest.main()
