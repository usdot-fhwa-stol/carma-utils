#!/usr/bin/python
#-*- coding: utf-8 -*-
import numpy as np

from SensedObject import SensedObject


# TODO Python funciton memoization could help.?

class SimulatedSensor:

    def __init__(self, config, carla_world, carla_sensor, noise_model):
        self.config = config
        self.carla_world = carla_world
        self.carla_sensor = carla_sensor
        self.noise_model = noise_model
        self.helper = SimulatedSensorHelper()

    def get_sensed_objects_in_frame(self):

        # Note: actors is the "driving" list indicating which items are considered inside the sensor FOV throughout

        # Get sensor including current location and configured sensor parameters
        sensor = self.helper.get_sensor()

        # Get sensed_object truth states from simulation
        sensed_objects = self.helper.get_scene_sensed_objects()

        # Prefilter
        sensed_objects = self.helper.prefilter(sensor, sensed_objects)

        # Get LIDAR hitpoints with Actor ID associations
        hitpoints = self.helper.get_carla_lidar_hitpoints()
        hitpoints = self.helper.get_hitpoints_sampled(hitpoints)
        hitpoints = self.helper.associate_hitpoints(hitpoints, sensed_objects)

        # Compute data needed for occlusion operation
        actor_angular_extents = self.helper.compute_actor_angular_extents(sensor, sensed_objects)
        detection_thresholds = self.helper.compute_adjusted_detection_thresholds(sensor, sensed_objects)

        # Apply occlusion
        sensed_objects = self.helper.apply_occlusion(sensed_objects, actor_angular_extents, hitpoints, detection_thresholds)

        return sensed_objects





class SimulatedSensorHelper:

    # ------------------------------------------------------------------------------
    # CARLA Scene SensedObject Retrieval
    # ------------------------------------------------------------------------------

    def get_sensor(self, carla_sensor):

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

    def get_scene_sensed_objects(self, carla_world, simulated_sensor_config):
        actors = carla_world.get_actors()
        return map(lambda actor: SensedObject(simulated_sensor_config, actor), actors)

    # ------------------------------------------------------------------------------
    # Prefilter
    # ------------------------------------------------------------------------------

    def prefilter(sensor, sensed_objects):

        # Filter by sensed_object type
        # Actor.type_id and Actor.semantic_tags are available for determining type; semantic_tags effectively specifies the type of sensed_object
        # Possible types are listed in the CARLA documentation: https://carla.readthedocs.io/en/0.9.10/ref_sensors/#semantic-segmentation-camera
        actors = filter(lambda actor: actor.semantic_tags), actors)
        self.config.prefilter.allowed_semantic_tags

        get_carla_lidar_hitpoints
        # Filter by radius
        simulated_sensor_config.prefilter.max_distance_meters


        pass

    # ------------------------------------------------------------------------------
    # CARLA Raw Sensor Data Retrieval
    # ------------------------------------------------------------------------------

    def get_carla_lidar_hitpoints(self, ):

    def get_hitpoints_sampled(self, hitpoints):

    def associate_hitpoints(hitpoints, sensed_objects):

    # ------------------------------------------------------------------------------
    # Computations
    # ------------------------------------------------------------------------------

    def compute_actor_angular_extents(self, sensor, sensed_objects):

    def compute_adjusted_detection_thresholds(self, sensor, sensed_objects):

    # ------------------------------------------------------------------------------
    # Occlusion Filter
    # ------------------------------------------------------------------------------

    def apply_occlusion(self, sensed_objects, actor_angular_extents, hitpoints, detection_thresholds):



