# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import carla
import numpy as np
from scipy.spatial.transform import Rotation


class CarlaUtils:
    """
    Generic CARLA utility functions.
    """

    #CARLA 0.9.10 type lookup table for van, cycle, motorcycle, truck
    vehicle_lookup_table = {'carlacola': "TRUCK", 
                            'cybertruck': "TRUCK", 
                            't2': "VAN", 
                            'low_rider': "MOTORCYCLE",
                            'ninja': "MOTORCYCLE",
                            'yzf': "MOTORCYCLE",
                            'crossbike': "CYCLIST",
                            'century': "CYCLIST",
                            'omafiets': "CYCLIST"}

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
    def get_actor_roll_pitch_yaw(carla_actor):
        """
        Get the orientation for an actor in order of roll, pitch, yaw.
        :param carla_actor: The carla.Actor to obtain data from.
        :return: numpy.array containing roll, pitch, yaw in radians.
        """
        carla_rotation = carla_actor.get_transform().rotation
        rotation_angles_deg = np.array([carla_rotation.roll, carla_rotation.pitch, carla_rotation.yaw])
        rotation_angles = np.deg2rad(rotation_angles_deg)
        return rotation_angles

    @staticmethod
    def get_actor_bounding_box_points(carla_actor):
        """
        Get all corners for an actor's bounding box, in the world frame.
        :param carla_actor: The carla.Actor to obtain data from.
        :return: List of numpy.array containing the bounding box points in the world frame.
        """

        try:
            bounding_box = carla_actor.bounding_box
        except AttributeError:
            raise AttributeError("There is no bounding_box attribute, in 0.9.10 only Pedestrian and Vehicles have this attribute, please check the input...")

        bounding_box_locations = bounding_box.get_world_vertices(carla_actor.get_transform())
        return [CarlaUtils.vector3d_to_numpy(location) for location in bounding_box_locations]

    @staticmethod
    def determine_object_type(carla_actor, allowed_semantic_tags):
        """
        Check for identification as one of the accepted types, and mark unidentified otherwise.
        :param carla_actor: The carla.Actor to obtain data from.
        :param allowed_semantic_tags: List of semantic tags which are allowed to be detected by the sensor.
        :return: The object type, or NONE if not in the allowed list.
        """
        #using type_id instead of semantic_tags
        #issue with semantic_tags in version 0.9.10:https://github.com/carla-simulator/carla/issues/2161

        temp_id_list =  carla_actor.type_id.split(".")
        temp_id = temp_id_list[0]
        if temp_id == "vehicle":
            vehicle_type = temp_id_list[2]
            #for object_type other than CAR
            if vehicle_type in CarlaUtils.vehicle_lookup_table.keys():
                return CarlaUtils.vehicle_lookup_table[vehicle_type]
            else:
                return "CAR"
        elif temp_id == "walker":
            return "PEDESTRIAN"
        else:
            return "NONE"

    @staticmethod
    def get_transform(sensor_position, sensor_rotation):
        """
        Get a carla.Transform from a position and rotation.
        :param sensor_position: Sensor position vector in CARLA world coordinates, represented as an array of floats.
        :param sensor_rotation: Sensor rotation in degrees, represented as an array of floats.
        :return: carla.Transform object.
        """
        if (isinstance(sensor_position, list) or isinstance(sensor_position, np.ndarray)) and len(sensor_position) >= 3:
            position = CarlaUtils.get_location(sensor_position)
        elif isinstance(sensor_position, carla.Location):
            position = sensor_position
        elif isinstance(sensor_position, carla.libcarla.Vector3D):
            position = carla.Location(x=sensor_position.x, y=sensor_position.y, z=sensor_position.z)
        else:
            raise ValueError("sensor_position must be a list of floats or a carla.Location object.")

        if (isinstance(sensor_rotation, list) or isinstance(sensor_position, np.ndarray)) and len(sensor_rotation) >= 3:
            rotation = CarlaUtils.get_rotation(sensor_rotation)
        elif isinstance(sensor_rotation, carla.Rotation):
            rotation = sensor_rotation
        else:
            raise ValueError("sensor_rotation must be a list of floats or a carla.Rotation object.")

        return carla.Transform(position, rotation)

    @staticmethod
    def get_location(position_vector):
        """
        Get a carla.Location from a position vector.
        :param position_vector: Position vector in CARLA world coordinates, represented as an array of floats.
        :return: carla.Location object.
        """
        return carla.Location(x=position_vector[0], y=position_vector[1], z=position_vector[2])

    @staticmethod
    def get_rotation(rotation_vector):
        """
        Get a carla.Rotation from a rotation vector.
        :param rotation_vector: Rotation vector in degrees, represented as an array of floats.
        :return: carla.Rotation object.
        """
        return carla.Rotation(pitch=rotation_vector[0], yaw=rotation_vector[1], roll=rotation_vector[2])

    @staticmethod
    def get_actor(carla_world, actor_id):
        """
        Get a carla.Actor from an actor ID.
        :param actor_id: The actor ID to look up.
        :return: carla.Actor object.
        """
        if not isinstance(actor_id, int) or actor_id < 0:
            return None

        return carla_world.get_actor(actor_id)
