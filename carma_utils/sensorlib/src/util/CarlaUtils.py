# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import numpy as np
from scipy.spatial.transform import Rotation


class CarlaUtils:
    """
    Generic CARLA utility functions.
    """

    @staticmethod
    def vector3d_to_numpy(vec):
        """
        Convert a carla.Vector3D to a numpy array.
        :param vec: carla.Vector3D to convert.
        :return: Vector in numpy array form.
        """
        return np.array([vec.x, vec.y, vec.z])

    @staticmethod
    def vector2d_to_numpy(vec):
        """
        Convert a carla.Vector2D to a numpy array.
        :param vec: carla.Vector2D to convert.
        :return: Vector in numpy array form.
        """
        return np.array([vec.x, vec.y])

    @staticmethod
    def get_actor_angular_velocity(carla_actor):
        """
        Get carla.Actor angular velocity in radians per second.
        :param carla_actor: The carla.Actor to obtain data from.
        :return: numpy.array containing angular velocity vector in radians per second.
        """
        angular_velocity_degpersecond = CarlaUtils.vector3d_to_numpy(carla_actor.get_angular_velocity())
        return np.deg2rad(angular_velocity_degpersecond)

    @staticmethod
    def get_actor_rotation_matrix(carla_actor):
        """
        Get the rotation matrix for an actor.
        :param carla_actor: The carla.Actor to obtain data from.
        :return: numpy.array containing the rotation matrix in radians.
        """
        carla_rotation = carla_actor.get_transform().rotation
        rotation_angles_deg = np.array([carla_rotation.roll, carla_rotation.pitch, carla_rotation.yaw])
        rotation_angles = np.deg2rad(rotation_angles_deg)
        rotation_matrix = Rotation.from_euler('xyz', rotation_angles)
        return rotation_matrix.as_matrix()

    @staticmethod
    def get_actor_bounding_box_points(carla_actor):
        """
        Get all corners for an actor's bounding box, in the world frame.
        :param carla_actor: The carla.Actor to obtain data from.
        :return: List of numpy.array containing the bounding box points in the world frame.
        """
        bounding_box_locations = carla_actor.get_world_vertices(carla_actor.get_transform())
        return [CarlaUtils.vector3d_to_numpy(location) for location in bounding_box_locations]

    @staticmethod
    def determine_object_type(carla_actor, allowed_semantic_tags):
        """
        Check for identification as one of the accepted types, and mark unidentified otherwise.
        :param carla_actor: The carla.Actor to obtain data from.
        :param allowed_semantic_tags: List of semantic tags which are allowed to be detected by the sensor.
        :return: The object type, or Unknown if not in the allowed list.
        """

        # Set intersection, except order matters
        for tag in carla_actor.semantic_tags:
            if tag in allowed_semantic_tags:
                return tag
        return "Unknown"
