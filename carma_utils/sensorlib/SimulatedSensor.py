#!/usr/bin/python
#-*- coding: utf-8 -*-
import itertools

import numpy
import numpy as np

from SensedObject import SensedObject
from SensorDataCollector import SensorDataCollector


class SimulatedSensor:

    def __init__(self, config, carla_world, carla_sensor, noise_model):
        self.__config = config
        self.__carla_world = carla_world
        self.__carla_sensor = carla_sensor
        self.__noise_model = noise_model
        self.__sensor_utilities = SimulatedSensorUtilities()

        # Data collection objects
        self.__raw_sensor_data_collector = SensorDataCollector(carla_world, carla_sensor)

    def get_sensed_objects_in_frame(self):

        # Note: actors is the "driving" list indicating which items are considered inside the sensor FOV throughout

        # Get sensor including current location and configured sensor parameters
        sensor = self.__sensor_utilities.get_sensor()

        # Get sensed_object truth states from simulation
        sensed_objects = self.__sensor_utilities.get_scene_sensed_objects()

        # Prefilter
        sensed_objects = self.__sensor_utilities.prefilter(sensor, sensed_objects)

        # Get LIDAR hitpoints with Actor ID associations
        hitpoints = self.__raw_sensor_data_collector.get_carla_lidar_hitpoints()
        hitpoints = self.__sensor_utilities.get_hitpoints_sampled(hitpoints)
        hitpoints = self.__sensor_utilities.associate_hitpoints(hitpoints, sensed_objects)

        # Compute data needed for occlusion operation
        actor_angular_extents = self.__sensor_utilities.compute_actor_angular_extents(sensor, sensed_objects)
        detection_thresholds = self.__sensor_utilities.compute_adjusted_detection_thresholds(sensor, sensed_objects)

        # Apply occlusion
        sensed_objects = self.__sensor_utilities.apply_occlusion(sensed_objects, actor_angular_extents, hitpoints, detection_thresholds)

        return sensed_objects





class SimulatedSensorUtilities:

    # ------------------------------------------------------------------------------
    # CARLA Scene SensedObject Retrieval
    # ------------------------------------------------------------------------------

    def get_sensor(self, carla_sensor):

        sensor = {}

        # Static sensor settings
        sensor["channels"] = carla_sensor["channels"]
        sensor["points_per_second"] = carla_sensor["points_per_second"]
        sensor["rotation_frequency"] = carla_sensor["rotation_frequency"]
        sensor["upper_fov"] = carla_sensor["upper_fov"]
        sensor["lower_fov"] = carla_sensor["lower_fov"]
        sensor["sensor_tick"] = carla_sensor["sensor_tick"]
        sensor["fov_angular_width"] = 0

        # Dynamic position and parameters
        sensor["position"] = self.__carla_sensor.get_location() as np.ndarray
        sensor["rotation"] = self.__carla_sensor.get_transform().get_matrix() as np.ndarray

        return sensor

    def get_scene_sensed_objects(self, carla_world, simulated_sensor_config):
        actors = carla_world.get_actors()
        return map(lambda actor: SensedObject(simulated_sensor_config, actor), actors)

    # ------------------------------------------------------------------------------
    # Prefilter
    # ------------------------------------------------------------------------------

    def prefilter(self, sensor, sensed_objects):

        # Filter by sensed_object type
        # Actor.type_id and Actor.semantic_tags are available for determining type; semantic_tags effectively specifies the type of sensed_object
        # Possible types are listed in the CARLA documentation: https://carla.readthedocs.io/en/0.9.10/ref_sensors/#semantic-segmentation-camera
        sensed_objects = filter(lambda obj: obj.object_type in self.__config.prefilter.allowed_semantic_tags, sensed_objects)

        # Filter by radius
        sensed_objects = filter(lambda obj: numpy.linalg.norm(obj.position - sensor.position) <= self.__config.prefilter.max_distance_meters,
                                sensed_objects)

        return sensed_objects

    # ------------------------------------------------------------------------------
    # Computations
    # ------------------------------------------------------------------------------

    def compute_actor_angular_extents(self, sensor, sensed_objects):
        return dict([ (sensed_object.id,
                       self.__compute_actor_angular_extent(sensor, sensed_object)) for sensed_object in sensed_objects ])

    def __compute_actor_angular_extent(self, sensor, sensed_object):
        bbox = sensed_object.bbox
        corner_vec = bbox.extent as np.ndarray
        all_corner_vectors = map(lambda X: np.matmul(np.diagflat(X), corner_vec), itertools.product([-1,1], repeat=3))
        thetas = map(lambda v: self.__compute_view_angle(sensor, v), all_corner_vectors)
        return (min(thetas), max(thetas))

    def __compute_view_angle(self, relative_object_position_vectors):
        return np.arccos(np.vdot(sensor.position, v) / (norm))

    def compute_adjusted_detection_thresholds(self, sensed_objects, relative_object_position_vectors):
        return dict([ (sensed_object.id,
                self.__compute_adjusted_detection_threshold(relative_object_position_vector)) for relative_object_position_vector in relative_object_position_vectors ])

    def __compute_adjusted_detection_threshold(self, relative_object_position_vector):
        r = self.__compute_range(relative_object_position_vector)
        dt_dr = self.__config["detection_threshold_scaling_formula"]["hitpoint_detection_ratio_threshold_per_meter_change_rate"]
        t_nominal = self.__config["detection_threshold_scaling_formula"]["nominal_hitpoint_detection_ratio_threshold"]
        return dt_dr * r * t_nominal

    def __compute_range(self, relative_object_position_vector):
        return numpy.linalg.norm(relative_object_position_vector)
    # return numpy.linalg.norm(sensed_object["position"] - sensor["position"])

    # ------------------------------------------------------------------------------
    # Occlusion Filter
    # ------------------------------------------------------------------------------

    def apply_occlusion(self, sensed_objects, actor_angular_extents, sensor, hitpoints, detection_thresholds):
        return filter(lambda obj: self.__is_visible(obj, actor_angular_extents[obj.id], hitpoints, detection_thresholds), sensed_objects)

    def __is_visible(self, sensed_object, actor_angular_extent, sensor, hitpoints, detection_thresholds):

        # Compute threshold hitpoint count for this object
        num_expected_hitpoints = self.__compute_expected_num_hitpoints(actor_angular_extent, sensor)
        detection_threshold_ratio = detection_thresholds[sensed_object.id]
        min_hitpoint_count = detection_threshold_ratio * num_expected_hitpoints

        # Compare hitpoint count
        num_hitpoints = len(hitpoints[sensed_object.id])

        return num_hitpoints >= min_hitpoint_count

    def __compute_expected_num_hitpoints(self, actor_angular_extent, sensor):
        num_points_per_scan = sensor.points_per_second / sensor.rotation_frequency
        theta_resolution = sensor.fov_angular_width / num_points_per_scan
        return (actor_angular_extent[1] - actor_angular_extent[0]) / theta_resolution



