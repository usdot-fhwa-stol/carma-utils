import unittest
from unittest.mock import MagicMock
import numpy as np

from src.util.CarlaUtils import CarlaUtils


class TestCarlaUtils(unittest.TestCase):

    def test_vector3d_to_numpy(self):
        vec = MagicMock(x=1.0, y=2.0, z=3.0)
        result = CarlaUtils.vector3d_to_numpy(vec)
        self.assertTrue(np.array_equal(result, np.array([1.0, 2.0, 3.0])))

    def test_vector2d_to_numpy(self):
        vec = MagicMock(x=1.0, y=2.0)
        result = CarlaUtils.vector2d_to_numpy(vec)
        self.assertTrue(np.array_equal(result, np.array([1.0, 2.0])))

    def test_determine_object_type(self):
        allowed_semantic_tags = ["Pedestrian", "Vehicle"]
        carla_actor = MagicMock(semantic_types=["Pedestrian"])
        result = CarlaUtils.determine_object_type(allowed_semantic_tags, carla_actor)
        self.assertEqual(result, "Pedestrian")

    def test_determine_object_type_unknown(self):
        allowed_semantic_tags = ["Pedestrian", "Vehicle"]
        carla_actor = MagicMock(semantic_types=["Cyclist"])
        result = CarlaUtils.determine_object_type(allowed_semantic_tags, carla_actor)
        self.assertEqual(result, "Unknown")

    def test_get_actor_angular_velocity(self):
        carla_actor = MagicMock(get_angular_velocity=MagicMock(return_value=MagicMock(x=1.0, y=2.0, z=3.0)))
        result = CarlaUtils.get_actor_angular_velocity(carla_actor)
        self.assertTrue(np.array_equal(result, np.deg2rad(np.array([1.0, 2.0, 3.0]))))

    def test_get_actor_rotation_matrix(self):
        carla_actor = MagicMock(get_transform=MagicMock(return_value=MagicMock(
            get_matrix=MagicMock(return_value=[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]))))
        result = CarlaUtils.get_actor_rotation_matrix(carla_actor)
        self.assertTrue(
            np.array_equal(result, np.deg2rad(np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]))))

    def test_get_actor_bounding_size(self):
        carla_actor = MagicMock(
            get_bounding_box=MagicMock(return_value=MagicMock(extent=MagicMock(x=1.0, y=2.0, z=3.0))))
        result = CarlaUtils.get_actor_bounding_size(carla_actor)
        self.assertTrue(np.array_equal(result, np.array([2.0, 4.0, 6.0])))


if __name__ == "__main__":
    unittest.main()
