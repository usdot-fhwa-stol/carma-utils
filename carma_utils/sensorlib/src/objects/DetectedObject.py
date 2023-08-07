#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np

from src.util.CarlaUtils import CarlaUtils


class DetectedObject:
    def __init__(self, carla_actor, object_type):
        self.__carla_actor = carla_actor
        self.__id = carla_actor.id
        self.__object_type = object_type
        self.__size = CarlaUtils.get_actor_bounding_size(carla_actor)  # Length, width, height of object in meters
        self.__position = CarlaUtils.vector3d_to_numpy(self.__carla_actor.get_location())
        self.__rotation = CarlaUtils.get_actor_rotation_matrix(carla_actor)

    def __init__(self, obj):
        self.__carla_actor = obj.carla_actor

    def get_id(self):
        return self.__id

    def get_object_type(self):
        return self.__object_type

    def get_position(self):
        return self.__position

    def get_rotation(self):
        return self.__rotation

    def get_velocity(self):
        return CarlaUtils.vector3d_to_numpy(self.__carla_actor.get_velocity())

    def get_angular_velocity(self):
        return CarlaUtils.get_actor_angular_velocity(self.__carla_actor)

    def get_position_covariance(self):
        # Use stand-in values which assume complete certainty
        return np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    def get_velocity_covariance(self):
        # Use stand-in values which assume complete certainty
        return np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    def get_confidence(self):
        # Use stand-in values which assume complete certainty
        return 1.0

    def get_size(self):
        return self.__size
