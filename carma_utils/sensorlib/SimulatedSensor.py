#!/usr/bin/python
#-*- coding: utf-8 -*-
from random import sample



class SimulatedSensor:

    # TODO config must contain prefilter threshold
    # TODO config must contain nominal hitpoint detection ratio threshold, and scaling parameters for the adjustable threshold

    def __init__(self, config, carla_world, carla_sensor, noise_model):
        self.carla_world = carla_world
        self.carla_sensor = None
        self.noise_model = None

    def get_objects_in_frame(self):

        # Note: actors is the "driving" list indicating which items are considered inside the sensor FOV throughout

        # Get sensors including locations and parameters
        sensor_info = self.__get_carla_sensor_info()

        # Get Actors including locations
        actors = self.__get_carla_actors()

        # Prefilter
        actors = self.__prefilter(sensor_info, actors)

        # Get LIDAR hitpoints with Actor ID associations
        hitpoints = self.__get_carla_lidar_hitpoints()
        hitpoints = self.__sample_hitpoints(hitpoints)
        hitpoints = self.__associate(hitpoints, actors)

        # Compute data needed for occlusion operation
        actor_angular_extents = self.__compute_actor_angular_extents(sensor_info, actors)
        detection_thresholds = self.__compute_adjusted_detection_thresholds(sensor_info, actors)

        # Apply occlusion
        actors = self.__apply_occlusion(actors, actor_angular_extents, hitpoints, detection_thresholds)

        return actors






    def __get_carla_sensor_info(self):
        pass


    def __get_carla_actors(self):
        pass


    def __prefilter(self, sensor_info, actors):
        pass


    def __get_carla_lidar_hitpoints(self):
        pass


    def __associate(self, hitpoints, actors):
        pass


    def __compute_actor_angular_extents(self, sensor_info, actors):
        pass


    def __compute_adjusted_detection_thresholds(self, sensor_info, actors):
        pass


    def __apply_occlusion(self, actors, actor_angular_extents, hitpoints, detection_thresholds):
        pass






