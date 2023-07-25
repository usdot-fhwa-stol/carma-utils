#!/usr/bin/python
#-*- coding: utf-8 -*-
import numpy as np

from src.util.CarlaActorUtils import CarlaActorUtils


# TODO Do we want getters and setters?

class SensedObject:
    def __init__(self, simulated_sensor_config, carla_actor):
        self.id = carla_actor.id
        self.object_type = SensedObject.determine_object_type(simulated_sensor_config, carla_actor)

        self.position = None
        self.rotation = None
        self.velocity = None
        self.angular_velocity = CarlaActorUtils.get_actor_angular_velocity()

        self.position_covariance = None
        self.velocity_covariance = None

        self.size = None
        self.confidence = 1.0  # Default

    @staticmethod
    def determine_object_type(simulated_sensor_config, carla_actor):
        """
        Check for identification as one of the accepted types, and mark unidentified otherwise.
        Args:

        """

        # Set intersection, except order matters
        for tag in simulated_sensor_config["prefilter"]["allowed_semantic_tags"]:
            if tag in carla_actor.semantic_types:
                return tag
        return "Unknown"

