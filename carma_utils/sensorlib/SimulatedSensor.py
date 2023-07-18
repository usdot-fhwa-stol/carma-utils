#!/usr/bin/python
#-*- coding: utf-8 -*-
import numpy as np

# TODO Python funciton memoization could help.?

class SimulatedSensor:

    def __init__(self, config, carla_world, carla_sensor, noise_model):
        self.config = config
        self.carla_world = carla_world
        self.carla_sensor = None
        self.noise_model = None
        self.helper = SimulatedSensorHelper()

    def get_objects_in_frame(self):

        # Note: actors is the "driving" list indicating which items are considered inside the sensor FOV throughout

        # Get sensors including locations and parameters
        sensor = self.helper.get_carla_sensor_info()

        # Get Actors including locations
        actors = self.helper.get_carla_actors()

        # Prefilter
        actors = self.helper.prefilter(sensor, actors)

        # Get LIDAR hitpoints with Actor ID associations
        hitpoints = self.helper.get_carla_lidar_hitpoints()
        hitpoints = self.helper.sample_hitpoints(hitpoints)
        hitpoints = self.helper.associate(hitpoints, actors)

        # Compute data needed for occlusion operation
        actor_angular_extents = self.helper.compute_actor_angular_extents(sensor_info, actors)
        detection_thresholds = self.helper.compute_adjusted_detection_thresholds(sensor_info, actors)

        # Apply occlusion
        actors = self.helper.apply_occlusion(actors, actor_angular_extents, hitpoints, detection_thresholds)

        return actors





class SimulatedSensorHelper:

    def get_carla_sensor_info(self, carla_sensor):

        sensor_info = {}

        # Static sensor settings
        sensor_info["channels"] = carla_sensor["channels"]
        sensor_info["points_per_second"] = carla_sensor["points_per_second"]
        sensor_info["rotation_frequency"] = carla_sensor["rotation_frequency"]
        sensor_info["upper_fov"] = carla_sensor["upper_fov"]
        sensor_info["lower_fov"] = carla_sensor["lower_fov"]
        sensor_info["horizontal_fov"] = carla_sensor["horizontal_fov"]
        sensor_info["sensor_tick"] = carla_sensor["sensor_tick"]

        # Dynamic position and parameters
        sensor_info["position"] = carla_sensor.get_location() as np.ndarray
        sensor_info["rotation"] = carla_sensor.get_transform().get_matrix() as np.ndarray

        return sensor_info


    def get_carla_actors(self):
        # actors = get_actors()
        # foreach (actor in actors):
        # id = actor.id
        # position = actor.bounding_box.location
        # bounding_box = actor.bounding_box.extent
        pass


    def prefilter(self, sensor_info, actors):

        # Filter by object type
        # Actor.type_id and Actor.semantic_tags are available for determining type; semantic_tags effectively specifies the type of object
        # Possible types are listed in the CARLA documentation: https://carla.readthedocs.io/en/0.9.10/ref_sensors/#semantic-segmentation-camera
        is_tag_allowed = lambda tag: tag in self.config.prefilter.allowed_semantic_tags
        actors = filter(lambda actor: is_tag_allowed(actor.semantic_tags), actors)


get_carla_lidar_hitpoints
        # Filter by radius
        simulated_sensor_config.prefilter.max_distance_meters


        pass

    def get_carla_lidar_hitpoints(self):
        pass
    

    def associate(self, hitpoints, actors):
        pass


    def compute_actor_angular_extents(self, sensor_info, actors):
        pass


    def compute_adjusted_detection_thresholds(self, sensor_info, actors):
        pass


    def apply_occlusion(self, actors, actor_angular_extents, hitpoints, detection_thresholds):
        pass






