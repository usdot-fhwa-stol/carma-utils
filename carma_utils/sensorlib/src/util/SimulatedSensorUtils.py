import itertools

import numpy as np
import yaml

from src.objects.DetectedObject import DetectedObject


class SimulatedSensorUtils:

    # ------------------------------------------------------------------------------
    # Configuration file reading
    # ------------------------------------------------------------------------------

    @staticmethod
    def load_config_from_file(config_filepath):
        with open(config_filepath, 'r') as file:
            config = yaml.safe_load(file)
            return config

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
    def prefilter(config, sensor, detected_objects):
        # Filter by detected_object type
        # Actor.type_id and Actor.semantic_tags are available for determining type; semantic_tags effectively specifies the type of detected_object
        # Possible types are listed in the CARLA documentation: https://carla.readthedocs.io/en/0.9.10/ref_sensors/#semantic-segmentation-camera
        detected_objects = filter(lambda obj: obj.object_type in config.prefilter.allowed_semantic_tags,
                                  detected_objects)

        # Filter by radius
        detected_objects = filter(lambda obj: np.linalg.norm(
            obj.position - sensor.position) <= config.prefilter.max_distance_meters,
                                  detected_objects)

        return detected_objects

    # ------------------------------------------------------------------------------
    # Computations
    # ------------------------------------------------------------------------------

    @staticmethod
    def compute_actor_angular_extents(sensor, detected_objects):
        return dict([(detected_object.id,
                      SimulatedSensorUtils.compute_actor_angular_extent(sensor, detected_object)) for detected_object in detected_objects])

    @staticmethod
    def compute_actor_angular_extent(sensor, detected_object):
        bbox = detected_object.bbox
        corner_vec = np.array(bbox.extent)
        all_corner_vectors = map(lambda X: np.matmul(np.diagflat(X), corner_vec), itertools.product([-1, 1], repeat=3))
        thetas = map(lambda v: SimulatedSensorUtils.compute_view_angle(sensor, v), all_corner_vectors)
        return (min(thetas), max(thetas))

    @staticmethod
    def compute_view_angle(sensor, vec):
        return np.arccos(np.vdot(sensor.position, vec) / (np.linalg.norm(sensor.position) * np.linalg.norm(vec)))

    @staticmethod
    def compute_adjusted_detection_thresholds(config, sensor, detected_objects):
        return dict([(detected_object.id,
                      SimulatedSensorUtils.compute_adjusted_detection_threshold(config, detected_object.position - sensor.position)) for
                     detected_object in detected_objects])

    @staticmethod
    def compute_adjusted_detection_threshold(config, relative_object_position_vector):
        r = SimulatedSensorUtils.compute_range(relative_object_position_vector)
        dt_dr = config["detection_threshold_scaling_formula"][
            "hitpoint_detection_ratio_threshold_per_meter_change_rate"]
        t_nominal = config["detection_threshold_scaling_formula"]["nominal_hitpoint_detection_ratio_threshold"]
        # TODO Review this formula
        return dt_dr * r * t_nominal

    @staticmethod
    def compute_range(relative_object_position_vector):
        return np.linalg.norm(relative_object_position_vector)

    # return np.linalg.norm(detected_object["position"] - sensor["position"])

    # ------------------------------------------------------------------------------
    # Occlusion Filter
    # ------------------------------------------------------------------------------

    @staticmethod
    def apply_occlusion(detected_objects, actor_angular_extents, sensor, hitpoints, detection_thresholds):
        return filter(
            lambda obj: SimulatedSensorUtils.is_visible(obj, actor_angular_extents[obj.id], hitpoints, detection_thresholds),
            detected_objects)

    @staticmethod
    def is_visible(detected_object, actor_angular_extent, sensor, hitpoints, detection_thresholds):
        # Compute threshold hitpoint count for this object
        num_expected_hitpoints = SimulatedSensorUtils.compute_expected_num_hitpoints(actor_angular_extent, sensor)
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
