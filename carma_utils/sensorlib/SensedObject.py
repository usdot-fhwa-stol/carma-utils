#!/usr/bin/python
#-*- coding: utf-8 -*-

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

    @property
    def size(self):
        return self.size

    @size.setter
    def size(self, value):
        self.size = value

    @property
    def object_type(self):
        return self.object_type

    @object_type.setter
    def object_type(self, value):
        self.object_type = value

    @property
    def rotation(self):
        return self.rotation

    @rotation.setter
    def rotation(self, value):
        self.rotation = value

    @property
    def confidence(self):
        return self.confidence

    @confidence.setter
    def confidence(self, value):
        self.confidence = value

    @property
    def covariance(self):
        return self.covariance

    @covariance.setter
    def covariance(self, value):
        self.covariance = value

    @property
    def id(self):
        return self.id

    @id.setter
    def id(self, value):
        self.id = value

    @property
    def position(self):
        return self.position

    @position.setter
    def position(self, value):
        self.position = value

    @property
    def velocity(self):
        return self.velocity

    @velocity.setter
    def velocity(self, value):
        self.velocity = value

    def __get_object_type(self, simulated_sensor_config, carla_actor):
        """
        Check for identification as the accepted types, and mark unidentified otherwise.
        Args:

        """
        for tag in simulated_sensor_config.prefilter.allowed_semantic_tags:
            if tag in carla_actor.semantic_types:
                return tag
        return "Unknown"
