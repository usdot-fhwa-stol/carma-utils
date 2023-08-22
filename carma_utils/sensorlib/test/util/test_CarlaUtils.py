# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import unittest

import carla
import numpy as np
from unittest.mock import MagicMock

from scipy.spatial.transform import Rotation

from src.util.CarlaUtils import CarlaUtils


class TestCarlaUtils(unittest.TestCase):

    def setUp(self):
        self.carla_actor = MagicMock()

    def test_vector3d_to_numpy(self):
        vec = MagicMock(x=1.0, y=2.0, z=3.0)
        result = CarlaUtils.vector3d_to_numpy(vec)
        self.assertTrue(np.array_equal(result, np.array([1.0, 2.0, 3.0])))

    def test_vector2d_to_numpy(self):
        vec = MagicMock(x=1.0, y=2.0)
        result = CarlaUtils.vector2d_to_numpy(vec)
        self.assertTrue(np.array_equal(result, np.array([1.0, 2.0])))

    def test_get_actor_angular_velocity(self):
        carla_actor = MagicMock(get_angular_velocity=MagicMock(return_value=MagicMock(x=1.0, y=2.0, z=3.0)))
        result = CarlaUtils.get_actor_angular_velocity(carla_actor)
        self.assertTrue(np.array_equal(result, np.deg2rad(np.array([1.0, 2.0, 3.0]))))

    def test_get_actor_rotation_matrix(self):
        # Replicate the rotation
        rotation_angles_deg = np.array([45.0, 30.0, 90.0])
        rotation_angles = np.deg2rad(rotation_angles_deg)
        rotation_matrix = Rotation.from_euler('xyz', rotation_angles)

        # Get the rotation matrix
        carla_actor = MagicMock(
            get_transform=MagicMock(return_value=MagicMock(rotation=carla.Rotation(30.0, 90.0, 45.0))))
        result = CarlaUtils.get_actor_rotation_matrix(carla_actor)

        assert np.allclose(result, rotation_matrix.as_matrix())

    def test_get_actor_bounding_box_points(self):
        carla_actor = MagicMock(get_world_vertices=MagicMock(return_value=[carla.Location(1.0, 2.0, 3.0),
                                                                           carla.Location(4.0, 5.0, 6.0),
                                                                           carla.Location(7.0, 8.0, 9.0),
                                                                           carla.Location(10.0, 11.0, 12.0)]))
        result = CarlaUtils.get_actor_bounding_box_points(carla_actor)
        assert (result[0] == np.array([1.0, 2.0, 3.0])).all()
        assert (result[1] == np.array([4.0, 5.0, 6.0])).all()
        assert (result[2] == np.array([7.0, 8.0, 9.0])).all()
        assert (result[3] == np.array([10.0, 11.0, 12.0])).all()

    def test_determine_object_type(self):
        # Nominal case
        carla_actor = MagicMock(semantic_tags=["Vehicle"])
        assert "Vehicle" == CarlaUtils.determine_object_type(carla_actor, ["Pedestrian", "Vehicle"])

        # Multiple types
        assert "Vehicle" == CarlaUtils.determine_object_type(carla_actor, ["Invalid Type", "Vehicle"])

        # No allowed type
        assert "NONE" == CarlaUtils.determine_object_type(carla_actor, ["Invalid Type"])

        # NONE
        carla_actor = MagicMock(semantic_tags=["Cyclist"])
        result = CarlaUtils.determine_object_type(carla_actor, ["Pedestrian", "Vehicle"])
        self.assertEqual(result, "NONE")


if __name__ == "__main__":
    unittest.main()
