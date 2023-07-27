#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np

from src.util.CarlaActorUtils import CarlaActorUtils
from src.util.CarlaUtils import CarlaUtils


class DetectedObject:
    def __init__(self, simulated_sensor_config, carla_actor):
        self.__carla_actor = carla_actor
        self.__id = carla_actor.id
        self.__object_type = CarlaActorUtils.determine_object_type(
            simulated_sensor_config["prefilter"]["allowed_semantic_tags"], carla_actor)
        self.__size = CarlaActorUtils.get_actor_bounding_size(carla_actor)  # Length, width, height of object in meters

    def get_id(self):
        return self.__id

    def get_object_type(self):
        return self.__object_type

    def get_position(self):
        return CarlaUtils.vector3d_to_numpy(self.__carla_actor.get_location())

    def get_rotation(self):
        return CarlaActorUtils.get_actor_rotation_matrix(self.__carla_actor)

    def get_velocity(self):
        return CarlaUtils.vector3d_to_numpy(self.__carla_actor.get_velocity())

    def get_angular_velocity(self):
        return CarlaActorUtils.get_actor_angular_velocity(self.__carla_actor)

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
