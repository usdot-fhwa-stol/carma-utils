#!/usr/bin/python
# -*- coding: utf-8 -*-
import itertools

import numpy
import numpy as np
import yaml

from DetectedObject import DetectedObject
from SensorDataCollector import SensorDataCollector
from src.objects.ProxySensor import ProxySensor


class SimulatedSensor:

    def __init__(self, carla_world, carla_sensor, noise_model):
        self.__carla_world = carla_world
        self.__carla_sensor = carla_sensor
        self.__noise_model = noise_model

        # Data collection objects
        self.__raw_sensor_data_collector = SensorDataCollector(carla_world, carla_sensor)

    def load_config_from_dict(self, config):
        self.__config = config

    def load_config_from_file(self, config_file_path):
        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)
        self.__config = config

    def get_detected_objects_in_frame(self):
        # Note: actors is the "driving" list indicating which items are considered inside the sensor FOV throughout

        # Get sensor including current location and configured sensor parameters
        sensor = ProxySensor(self.__carla_sensor)

        # Get detected_object truth states from simulation
        detected_objects = SimulatedSensorUtilities.get_scene_detected_objects()

        # Prefilter
        detected_objects = SimulatedSensorUtilities.prefilter(sensor, detected_objects)

        # Get LIDAR hitpoints with Actor ID associations
        hitpoints = self.__raw_sensor_data_collector.get_carla_lidar_hitpoints()

        # Compute data needed for occlusion operation
        actor_angular_extents = SimulatedSensorUtilities.compute_actor_angular_extents(sensor, detected_objects)
        detection_thresholds = SimulatedSensorUtilities.compute_adjusted_detection_thresholds(sensor, detected_objects)

        # Apply occlusion
        detected_objects = SimulatedSensorUtilities.apply_occlusion(detected_objects, actor_angular_extents, hitpoints,
                                                                  detection_thresholds)
        # Apply noise
        detected_objects = SimulatedSensorUtilities.apply_noise(detected_objects, self.__noise_model)

        return detected_objects


class SimulatedSensorUtilities:

    # ------------------------------------------------------------------------------
    # CARLA Scene DetectedObject Retrieval
    # ------------------------------------------------------------------------------

    @staticmethod
    def get_scene_detected_objects(carla_world, simulated_sensor_config):
        actors = carla_world.get_actors()
        return map(lambda actor: DetectedObject(simulated_sensor_config, actor), actors)

    # ------------------------------------------------------------------------------
    # Prefilter
    # ------------------------------------------------------------------------------

    @staticmethod
    def prefilter(sensor, detected_objects):
        # Filter by detected_object type
        # Actor.type_id and Actor.semantic_tags are available for determining type; semantic_tags effectively specifies the type of detected_object
        # Possible types are listed in the CARLA documentation: https://carla.readthedocs.io/en/0.9.10/ref_sensors/#semantic-segmentation-camera
        detected_objects = filter(lambda obj: obj.object_type in self.__config.prefilter.allowed_semantic_tags,
                                detected_objects)

        # Filter by radius
        detected_objects = filter(lambda obj: numpy.linalg.norm(
            obj.position - sensor.position) <= self.__config.prefilter.max_distance_meters,
                                detected_objects)

        return detected_objects

    # ------------------------------------------------------------------------------
    # Computations
    # ------------------------------------------------------------------------------

    @staticmethod
    def compute_actor_angular_extents(sensor, detected_objects):
        return dict([(detected_object.id,
                      self.__compute_actor_angular_extent(sensor, detected_object)) for detected_object in detected_objects])

    @staticmethod
    def compute_actor_angular_extent(sensor, detected_object):
        bbox = detected_object.bbox
        corner_vec = bbox.extent as np.ndarray
        all_corner_vectors = map(lambda X: np.matmul(np.diagflat(X), corner_vec), itertools.product([-1, 1], repeat=3))
        thetas = map(lambda v: self.__compute_view_angle(sensor, v), all_corner_vectors)
        return (min(thetas), max(thetas))

    @staticmethod
    def compute_view_angle(sensor, vec):
        return np.arccos(np.vdot(sensor.position, vec) / (norm(sensor.position) * norm(vec)))

    @staticmethod
    def compute_adjusted_detection_thresholds(config, sensor, detected_objects):
        return dict([(detected_object.id,
                      self.__compute_adjusted_detection_threshold(config, detected_object.position - sensor.position)) for
                     detected_object in detected_objects])

    @staticmethod
    def compute_adjusted_detection_threshold(config, relative_object_position_vector):
        r = self.__compute_range(relative_object_position_vector)
        dt_dr = self.__config["detection_threshold_scaling_formula"][
            "hitpoint_detection_ratio_threshold_per_meter_change_rate"]
        t_nominal = self.__config["detection_threshold_scaling_formula"]["nominal_hitpoint_detection_ratio_threshold"]
        # TODO Review this formula
        return dt_dr * r * t_nominal

    @staticmethod
    def compute_range(relative_object_position_vector):
        return numpy.linalg.norm(relative_object_position_vector)

    # return numpy.linalg.norm(detected_object["position"] - sensor["position"])

    # ------------------------------------------------------------------------------
    # Occlusion Filter
    # ------------------------------------------------------------------------------

    @staticmethod
    def apply_occlusion(detected_objects, actor_angular_extents, sensor, hitpoints, detection_thresholds):
        return filter(
            lambda obj: self.__is_visible(obj, actor_angular_extents[obj.id], hitpoints, detection_thresholds),
            detected_objects)

    @staticmethod
    def is_visible(detected_object, actor_angular_extent, sensor, hitpoints, detection_thresholds):
        # Compute threshold hitpoint count for this object
        num_expected_hitpoints = self.__compute_expected_num_hitpoints(actor_angular_extent, sensor)
        detection_threshold_ratio = detection_thresholds[detected_object.id]
        min_hitpoint_count = detection_threshold_ratio * num_expected_hitpoints

        # Compare hitpoint count
        num_hitpoints = len(hitpoints[detected_object.id])

        return num_hitpoints >= min_hitpoint_count

    @staticmethod
    def compute_expected_num_hitpoints(actor_angular_extent, sensor):
        num_points_per_scan = sensor.points_per_second / sensor.rotation_frequency
        theta_resolution = sensor.fov_angular_width / num_points_per_scan
        return (actor_angular_extent[1] - actor_angular_extent[0]) / theta_resolution

    # ------------------------------------------------------------------------------
    # Noise Filter
    # ------------------------------------------------------------------------------

    @staticmethod
    def apply_noise(detected_objects, noise_model):
        detected_objects = noise_model.apply_position_noise(detected_objects)
        detected_objects = noise_model.apply_orientation_noise(detected_objects)
        detected_objects = noise_model.apply_type_noise(detected_objects)
        detected_objects = noise_model.apply_list_inclusion_noise(detected_objects)
        return detected_objects
