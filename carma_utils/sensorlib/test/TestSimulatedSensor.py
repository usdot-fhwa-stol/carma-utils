import unittest
from unittest.mock import MagicMock
import numpy as np

from src.SimulatedSensor import SimulatedSensor, SimulatedSensorUtilities
from src.SensedObject import SensedObject
from src.SensorDataCollector import SensorDataCollector

class SimulatedSensorTestCase(unittest.TestCase):
    # TODO Not reviewed

    def setUp(self):
        self.config = {
            "prefilter": {
                "allowed_semantic_tags": ["Pedestrian", "Vehicle"],
                "max_distance_meters": 100
            }
        }
        self.carla_world = MagicMock()
        self.carla_sensor = MagicMock()
        self.noise_model = MagicMock()
        self.simulated_sensor = SimulatedSensor(self.config, self.carla_world, self.carla_sensor, self.noise_model)

    def test_get_sensed_objects_in_frame(self):
        # Mocking necessary objects and methods
        carla_lidar_hitpoints = MagicMock()
        self.simulated_sensor._SimulatedSensor__raw_sensor_data_collector.get_carla_lidar_hitpoints.return_value = carla_lidar_hitpoints

        sensed_objects = [SensedObject(self.config, MagicMock(id=1)), SensedObject(self.config, MagicMock(id=2))]
        SimulatedSensorUtilities.get_scene_sensed_objects = MagicMock(return_value=sensed_objects)

        # Test the method
        result = self.simulated_sensor.get_sensed_objects_in_frame()

        # Assertions
        self.assertEqual(result, sensed_objects)
        SimulatedSensorUtilities.get_scene_sensed_objects.assert_called_once()
        self.simulated_sensor._SimulatedSensor__raw_sensor_data_collector.get_carla_lidar_hitpoints.assert_called_once()

class SimulatedSensorUtilitiesTestCase(unittest.TestCase):

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
        self.carla_sensor.get_transform.return_value = MagicMock(get_matrix=MagicMock(return_value=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]))

        result = SimulatedSensorUtilities.get_sensor(self.carla_sensor)

        self.assertEqual(result["channels"], carla_sensor["channels"])
        self.assertEqual(result["points_per_second"], carla_sensor["points_per_second"])
        self.assertEqual(result["rotation_frequency"], carla_sensor["rotation_frequency"])
        self.assertEqual(result["upper_fov"], carla_sensor["upper_fov"])
        self.assertEqual(result["lower_fov"], carla_sensor["lower_fov"])
        self.assertEqual(result["sensor_tick"], carla_sensor["sensor_tick"])
        self.assertTrue(np.array_equal(result["position"], np.array([0.0, 0.0, 0.0])))
        self.assertTrue(np.array_equal(result["rotation"], np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])))

    def test_prefilter(self):
        sensor = {
            "position": np.array([0.0, 0.0, 0.0])
        }
        sensed_objects = [SensedObject({}, MagicMock(object_type="Pedestrian", position=np.array([1.0, 0.0, 0.0]))),
                          SensedObject({}, MagicMock(object_type="Vehicle", position=np.array([2.0, 0.0, 0.0]))),
                          SensedObject({}, MagicMock(object_type="Cyclist", position=np.array([3.0, 0.0, 0.0])))]
        config = {
            "prefilter": {
                "allowed_semantic_tags": ["Pedestrian", "Vehicle"],
                "max_distance_meters": 2.5
            }
        }

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
        sensed_objects = [SensedObject({}, MagicMock(id=1)), SensedObject({}, MagicMock(id=2))]
        relative_object_position_vectors = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

        result = SimulatedSensorUtilities.compute_adjusted_detection_thresholds(sensed_objects, relative_object_position_vectors)

        self.assertEqual(len(result), 2)
        self.assertEqual(result[1], 0.0)
        self.assertEqual
