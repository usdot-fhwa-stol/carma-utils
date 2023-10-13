# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import unittest
from unittest.mock import MagicMock

import carla
import numpy as np
from scipy.spatial.transform import Rotation

from src.objects.DetectedObject import DetectedObjectBuilder


class TestDetectedObject(unittest.TestCase):

    def setUp(self):
        self.carla_actor = MagicMock()
        self.carla_actor.id = 1
        self.carla_actor.semantic_tags = [int(carla.CityObjectLabel.Vehicles)]
        self.carla_actor.bounding_box = MagicMock(
            get_world_vertices=MagicMock(return_value=[carla.Location(1.0, 2.0, 3.0),
                                                       carla.Location(4.0, 5.0, 6.0),
                                                       carla.Location(7.0, 8.0, 9.0),
                                                       carla.Location(10.0, 11.0, 12.0)]))
        self.carla_actor.get_transform = MagicMock(return_value=MagicMock(rotation=carla.Rotation(30.0, 90.0, 45.0)))
        self.carla_actor.get_location = MagicMock(return_value=carla.Location(1.0, 2.0, 3.0))
        self.carla_actor.get_velocity = MagicMock(return_value=carla.Vector3D(4.0, 5.0, 6.0))
        self.carla_actor.get_angular_velocity = MagicMock(return_value=carla.Vector3D(7.0, 8.0, 9.0))

    def test_builder(self):
        detected_object = DetectedObjectBuilder.build_detected_object(self.carla_actor, ["Vehicles"])

        assert detected_object.carla_actor == self.carla_actor
        assert detected_object.id == 1
        assert detected_object.object_type == "Vehicles"

        # Bounding box
        assert (detected_object.bounding_box_in_world_coordinate_frame[0] == np.array([1.0, 2.0, 3.0])).all()
        assert (detected_object.bounding_box_in_world_coordinate_frame[1] == np.array([4.0, 5.0, 6.0])).all()
        assert (detected_object.bounding_box_in_world_coordinate_frame[2] == np.array([7.0, 8.0, 9.0])).all()
        assert (detected_object.bounding_box_in_world_coordinate_frame[3] == np.array([10.0, 11.0, 12.0])).all()

        # P,V
        assert (detected_object.position == np.array([1.0, 2.0, 3.0])).all()
        assert (detected_object.velocity == np.array([4.0, 5.0, 6.0])).all()

        # Replicate the rotation
        rotation_angles_deg = np.array([45.0, 30.0, 90.0])
        rotation_angles = np.deg2rad(rotation_angles_deg)
        rotation_matrix = Rotation.from_euler('xyz', rotation_angles)
        assert np.allclose(detected_object.rotation, rotation_matrix.as_matrix())

        # Angular velocity
        assert (detected_object.angular_velocity == np.deg2rad(np.array([7.0, 8.0, 9.0]))).all()

        # Uncertainty
        assert (detected_object.position_covariance == np.zeros((3, 3))).all()
        assert (detected_object.velocity_covariance == np.zeros((3, 3))).all()
        assert detected_object.confidence == 1.0
