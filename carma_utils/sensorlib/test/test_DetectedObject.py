import unittest
from unittest.mock import MagicMock

import carla
import numpy as np

from src.objects.DetectedObject import DetectedObject
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestDetectedObject(unittest.TestCase):

    def setUp(self):
        self.simulated_sensor_config = SimulatedSensorTestUtils.generate_simulated_sensor_config()

        # Mock the carla.Actor class
        self.carla_actor = MagicMock()
        self.carla_actor.id = 123
        self.carla_actor.attributes = dict()
        self.carla_actor.is_alive = True
        self.carla_actor.parent = None
        self.carla_actor.semantic_types = ["Vehicles"]
        self.carla_actor.type_id = "vehicle.ford.mustang"

        self.carla_actor.get_acceleration = MagicMock(return_value=carla.Vector3D(0.0, 0.0, 0.0))
        self.carla_actor.get_angular_velocity = MagicMock(return_value=carla.Vector3D(0.0, 0.0, 0.0))
        self.carla_actor.get_location = MagicMock(return_value=carla.Location(10.0, 15.0, 7.0))
        self.carla_actor.get_transform = MagicMock(
            return_value=carla.Transform(carla.Location(10.0, 15.0, 7.0), carla.Rotation(3.0, 1.4, 4.0)))
        self.carla_actor.get_velocity = MagicMock(return_value=carla.Vector3D(100.0, 1.0, 0.0))
        self.carla_actor.get_world = MagicMock(return_value=carla.World)

        self.carla_actor.get_bounding_box.return_value = MagicMock(extent=MagicMock(x=3.0, y=4.0, z=5.0))

        # Construct the DetectedObject
        self.detected_object = DetectedObject(self.carla_actor, "Vehicles")

    def test_get_id(self):
        self.assertEqual(self.detected_object.get_id(), 123)

    def test_get_object_type(self):
        self.assertEqual(self.detected_object.get_object_type(), "Vehicles")

    def test_get_position(self):
        expected_position = np.array([10.0, 15.0, 7.0])
        self.assertTrue(np.array_equal(self.detected_object.get_position(), expected_position))

    def test_get_rotation(self):
        expected_rotation = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        self.assertTrue(np.array_equal(self.detected_object.get_rotation(), expected_rotation))

    def test_get_velocity(self):
        expected_velocity = np.array([0.0, 0.0, 0.0])
        self.assertTrue(np.array_equal(self.detected_object.get_velocity(), expected_velocity))

    def test_get_angular_velocity(self):
        expected_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.assertTrue(np.array_equal(self.detected_object.get_angular_velocity(), expected_angular_velocity))

    def test_get_position_covariance(self):
        expected_covariance = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        self.assertTrue(np.array_equal(self.detected_object.get_position_covariance(), expected_covariance))

    def test_get_velocity_covariance(self):
        expected_covariance = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        self.assertTrue(np.array_equal(self.detected_object.get_velocity_covariance(), expected_covariance))

    def test_get_confidence(self):
        self.assertEqual(self.detected_object.get_confidence(), 1.0)

    def test_get_size(self):
        self.assertTrue(np.array_equal(self.detected_object.get_size(), np.array([3.0, 4.0, 5.0])))


if __name__ == '__main__':
    unittest.main()
