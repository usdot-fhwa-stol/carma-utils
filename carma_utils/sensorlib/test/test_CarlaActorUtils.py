import unittest
import numpy as np
from unittest.mock import MagicMock

from src.util.CarlaUtils import CarlaUtils
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestCarlaUtils(unittest.TestCase):

    def setUp(self):
        self.carla_actor = MagicMock()

    def test_determine_object_type(config, carla_actor):
        # Nominal case
        carla_actor.semantic_types = ["Vehicles"]
        assert "Vehicles" == CarlaUtils.determine_object_type(config, carla_actor)

        # Multiple types
        carla_actor.semantic_types = ["Invalid Type", "Vehicles"]
        assert "Vehicles" == CarlaUtils.determine_object_type(config, carla_actor)

        # No allowed type
        carla_actor.semantic_types = ["Invalid Type"]
        assert "Unknown" == CarlaUtils.determine_object_type(config, carla_actor)

    def test_get_actor_angular_velocity(self):
        expected_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.carla_actor.get_angular_velocity.return_value = MagicMock(x=0.0, y=0.0, z=0.0)
        result = CarlaUtils.get_actor_angular_velocity(self.carla_actor)
        self.assertTrue(np.array_equal(result, expected_angular_velocity))

    def test_get_actor_rotation_matrix(self):
        expected_rotation_matrix = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.carla_actor.get_transform.return_value = MagicMock(
            get_matrix=MagicMock(return_value=expected_rotation_matrix))
        result = CarlaUtils.get_actor_rotation_matrix(self.carla_actor)
        self.assertTrue(np.array_equal(result, expected_rotation_matrix))

    def test_get_actor_bounding_size(self):
        expected_bounding_size = np.array([2.0, 4.0, 6.0])
        self.carla_actor = MagicMock(get_actor_bounding_size=MagicMock(return_value=MagicMock(extent=MagicMock(
            x=1.0, y=2.0, z=3.0))))
        result = CarlaUtils.get_actor_bounding_size(self.carla_actor)
        self.assertTrue(np.all_close(result, expected_bounding_size, atol=SimulatedSensorTestUtils.TOLERANCE))


if __name__ == '__main__':
    unittest.main()
