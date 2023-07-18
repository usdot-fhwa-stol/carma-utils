#!/usr/bin/python
#-*- coding: utf-8 -*-

# TODO Do we want getters and setters?

class SensedObject:
    def __init__(self, simulated_sensor_config, carla_actor):
        self.id = carla_actor.id
        self.object_type = self.__get_object_type(carla_actor)
        self.position = None
        self.rotation = None
        self.covariance = None
        self.velocity = None
        self.size = None
        self.confidence = 1.0  # Default

    def __get_object_type(self, simulated_sensor_config, carla_actor):
        """
        Check for identification as one of the accepted types, and mark unidentified otherwise.
        Args:

        """
        for tag in simulated_sensor_config.prefilter.allowed_semantic_tags:
            if tag in carla_actor.semantic_types:
                return tag
        return "Unknown"
