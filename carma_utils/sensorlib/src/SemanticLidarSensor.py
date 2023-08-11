import itertools
from collections import deque
from dataclasses import replace

import numpy as np
import yaml

from src.SimulatedSensor import SimulatedSensor
from src.objects.DetectedObject import DetectedObject
from collections import Counter


class SemanticLidarSensor(SimulatedSensor):

    def __init__(self, config, sensor, data_collector, noise_model):
        self.__config = config

        # Fundamental internal objects
        self.__sensor = sensor
        self.__data_collector = data_collector
        self.__noise_model = noise_model

        # Structures to store reassociation information
        self.__actor_id_association = {}
        self.__trailing_id_associations = deque(
            maxlen=config["geometry_reassociation"]["trailing_id_associations_max_length"])
        self.__rng = np.random.default_rng()

    # ------------------------------------------------------------------------------
    # Operation
    # ------------------------------------------------------------------------------

    def get_detected_objects_in_frame(self):
        # Get detected_object truth states from simulation
        detected_objects = SimulatedSensorUtils.get_scene_detected_objects()

        # Build the following lookup structures for infomration related to the detection or relationship between Sensor and DetectedObject:
        #   - Range
        #   - Angular extents
        #   - Confidence

        # Prefilter
        detected_objects, object_ranges = SimulatedSensorUtils.prefilter(sensor, detected_objects)

        # Get LIDAR hitpoints with Actor ID associations
        hitpoints = self.__raw_sensor_data_collector.get_carla_lidar_hitpoints()

        # Compute data needed for occlusion operation
        actor_angular_extents = SimulatedSensorUtils.compute_actor_angular_extents(sensor, detected_objects)
        detection_thresholds = SimulatedSensorUtils.compute_adjusted_detection_thresholds(config, detected_objects,
                                                                                          object_ranges)

        # Geometry re-association
        downsampled_hitpoints = sample_hitpoints(hitpoints, config["geometry_reassociation"]["downsampling_factor"])
        instantaneous_actor_id_association = compute_instantaneous_actor_id_association(downsampled_hitpoints,
                                                                                        detected_objects)
        self.__actor_id_association = update_actor_id_association(instantaneous_actor_id_association,
                                                                  self.__trailing_id_associations)

        # Apply occlusion
        detected_objects = SimulatedSensorUtils.apply_occlusion(detected_objects, actor_angular_extents, hitpoints,
                                                                detection_thresholds)
        # Apply noise
        detected_objects = SimulatedSensorUtils.apply_noise(detected_objects, self.__noise_model)

        return detected_objects

    # ------------------------------------------------------------------------------
    # CARLA Scene DetectedObject Retrieval
    # ------------------------------------------------------------------------------

    def get_scene_detected_objects(carla_world, simulated_sensor_config):
        actors = carla_world.get_actors()
        return [DetectedObject(simulated_sensor_config, actor) for actor in actors]

    # ------------------------------------------------------------------------------
    # Prefilter
    # ------------------------------------------------------------------------------

    def prefilter(config, sensor, detected_objects):
        """
        Perform the following operations:
            1. Convert coordinates to sensor-centric frame.
            2. Compute range from sensor to object.
            3. Filter objects by allowed type.
            4. Filter objects beyond the configured range threshold.

        :param config:
        :param sensor:
        :param detected_objects:
        :return:
        """

        #

        # Filter by detected_object type
        # Actor.type_id and Actor.semantic_tags are available for determining type; semantic_tags effectively specifies the type of detected_object
        # Possible types are listed in the CARLA documentation: https://carla.readthedocs.io/en/0.9.10/ref_sensors/#semantic-segmentation-camera
        detected_objects = list(filter(lambda obj: obj.object_type in config.prefilter.allowed_semantic_tags,
                                       detected_objects))

        # Convert coordinates to sensor-centric frame
        detected_objects = [replace(obj, position=np.subtract(obj.position, sensor.position)) for obj in
                            detected_objects]

        # Compute ranges
        object_ranges = dict([(obj.get_id(), np.linalg.norm(obj.position)) for obj in detected_objects])

        # Filter by radius
        detected_objects = list(
            filter(lambda obj: object_ranges[obj.get_id()] <= config.prefilter.max_distance_meters, detected_objects))

        return detected_objects, object_ranges

    # ------------------------------------------------------------------------------
    # Computations
    # ------------------------------------------------------------------------------

    def compute_actor_angular_extents(sensor, detected_objects):
        return dict([(detected_object.id,
                      SimulatedSensorUtils.compute_actor_angular_extent(sensor, detected_object)) for detected_object in
                     detected_objects])

    def compute_actor_angular_extent(sensor, detected_object):
        bbox = detected_object.bbox
        corner_vec = np.array(bbox.extent)
        # Using combinatorics for conciseness
        all_corner_vectors = [np.matmul(np.diagflat(X), corner_vec) for X in itertools.product([-1, 1], repeat=3)]
        thetas = [SimulatedSensorUtils.compute_view_angle(sensor, v) for v in all_corner_vectors]
        return min(thetas), max(thetas)

    def compute_view_angle(sensor, vec):
        return np.arccos(np.vdot(sensor.position, vec) / (np.linalg.norm(sensor.position) * np.linalg.norm(vec)))

    def compute_adjusted_detection_thresholds(config, detected_objects, object_ranges):
        return dict([(detected_object.id,
                      SimulatedSensorUtils.compute_adjusted_detection_threshold(config,
                                                                                object_ranges[
                                                                                    detected_object.get_id()], ))
                     for
                     detected_object in detected_objects])

    def compute_adjusted_detection_threshold(config, relative_object_position_vector):
        r = SimulatedSensorUtils.compute_range(relative_object_position_vector)
        dt_dr = config["detection_threshold_scaling_formula"][
            "hitpoint_detection_ratio_threshold_per_meter_change_rate"]
        t_nominal = config["detection_threshold_scaling_formula"]["nominal_hitpoint_detection_ratio_threshold"]
        # TODO Review this formula
        return dt_dr * r * t_nominal

    def compute_range(relative_object_position_vector):
        return np.linalg.norm(relative_object_position_vector)

    # return np.linalg.norm(detected_object["position"] - sensor["position"])









    # ------------------------------------------------------------------------------
    # Geometry Re-Association
    # ------------------------------------------------------------------------------

    def sample_hitpoints(self, hitpoints, sample_size):
        """Randomly sample points inside each object's set of LIDAR hitpoints."""
        return dict([(obj_id, self.__rng.choice(object_hitpoints, sample_size)) for obj_id, object_hitpoints in
                     hitpoints.items()])

    def compute_instantaneous_actor_id_association(self, downsampled_hitpoints, scene_objects):
        """
        Need:
        - Hitpoint geometry
        - Hitpoint wrong association
        - Available actor ID's
        - Actor locations
        - Current association

        return: Actor ID association applicable to this time step based on geometry-based association algorithm.
        """

        # Compute nearest neighbor for each hitpoint
        direct_nearest_neighbors = dict([(obj_id, self.compute_closest_object_list(hitpoint_list, scene_objects)) for obj_id, hitpoint_list in downsampled_hitpoints.items()])

        # Vote within each dictionary key
        return dict([(obj_id, self.vote_closest_object(object_id_list)) for obj_id, object_id_list in direct_nearest_neighbors.items()])

    def compute_closest_object_list(self, hitpoints, scene_objects):
        return [self.compute_closest_object(hitpoint, scene_objects) for hitpoint in hitpoints]

    def compute_closest_object(self, hitpoint, scene_objects):
        # TODO This function is written inefficiently.
        import numpy as np
        from scipy.spatial import distance
        object_positions = [obj.position for obj in scene_objects]
        distances = distance.cdist([hitpoint], object_positions)[0]
        closest_index = np.argmin(distances)
        closest_distance = distances[closest_index]
        closest_object = scene_objects[closest_index]
        return closest_distance, closest_object

    def vote_closest_object(self, object_ids):
        # Determine the object with the highest number of votes
        return Counter(object_ids).most_common(1)[0][0]




    def update_actor_id_association(self, instantaneous_actor_id_association, trailing_id_associations):
        """
        Update the most recent association based on the current time step's instantaneously-derived association.
        """
        # TODO Should UKF be used?
        # For now the highest-voted id wins.

        # Extract all (from_id, to_id) pairs from all dictionaries
        combined = trailing_id_associations + instantaneous_actor_id_association
        all_items = [association.items() for association in combined]

        # Build a multimap


        # return dict([(from_id, vote(to_id_list)) for from_id, to_id_list in combined.items()])









    # ------------------------------------------------------------------------------
    # Occlusion Filter
    # ------------------------------------------------------------------------------

    def apply_occlusion(detected_objects, actor_angular_extents, sensor, hitpoints, detection_thresholds):
        return list(filter(
            lambda obj: SimulatedSensorUtils.is_visible(obj, actor_angular_extents[obj.id], sensor, hitpoints,
                                                        detection_thresholds),
            detected_objects))

    def is_visible(detected_object, actor_angular_extent, sensor, hitpoints, detection_thresholds):
        # Compute threshold hitpoint count for this object
        num_expected_hitpoints = SimulatedSensorUtils.compute_expected_num_hitpoints(actor_angular_extent, sensor)
        detection_threshold_ratio = detection_thresholds[detected_object.id]
        min_hitpoint_count = detection_threshold_ratio * num_expected_hitpoints

        # Compare hitpoint count
        num_hitpoints = len(hitpoints[detected_object.id])

        return num_hitpoints >= min_hitpoint_count

    def compute_expected_num_hitpoints(actor_angular_extent, sensor):
        num_points_per_scan = sensor.points_per_second / sensor.rotation_frequency
        theta_resolution = sensor.fov_angular_width / num_points_per_scan
        return (actor_angular_extent[1] - actor_angular_extent[0]) / theta_resolution

    # ------------------------------------------------------------------------------
    # Noise Filter
    # ------------------------------------------------------------------------------

    def apply_noise(detected_objects, noise_model):
        detected_objects = noise_model.apply_position_noise(detected_objects)
        detected_objects = noise_model.apply_orientation_noise(detected_objects)
        detected_objects = noise_model.apply_type_noise(detected_objects)
        detected_objects = noise_model.apply_list_inclusion_noise(detected_objects)
        return detected_objects