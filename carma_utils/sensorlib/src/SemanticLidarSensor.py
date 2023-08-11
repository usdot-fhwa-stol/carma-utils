import itertools
from collections import deque
from dataclasses import replace

import numpy as np
import yaml

from src.SimulatedSensor import SimulatedSensor
from src.objects.DetectedObject import DetectedObject
from collections import Counter


class SemanticLidarSensor(SimulatedSensor):

    def __init__(self, simulated_sensor_config, carla_sensor_config, carla_world, sensor, data_collector, noise_model):
        self.__simulated_sensor_config = simulated_sensor_config
        self.__carla_sensor_config = carla_sensor_config

        # Fundamental internal objects
        self.__carla_world = carla_world
        self.__sensor = sensor
        self.__data_collector = data_collector
        self.__noise_model = noise_model

        # Structures to store reassociation information
        self.__actor_id_association = {}
        trailing_id_associations_count = simulated_sensor_config.geometry_reassociation.trailing_id_associations_count
        self.__trailing_id_associations = deque([{}], maxlen=trailing_id_associations_count)
        self.__rng = np.random.default_rng()

    # ------------------------------------------------------------------------------
    # Operation
    # ------------------------------------------------------------------------------

    def get_detected_objects_in_frame(self):
        # Get detected_object truth states from simulation
        detected_objects = self.get_scene_detected_objects()

        # Build the following lookup structures for infomration related to the detection or relationship between Sensor and DetectedObject:
        #   - Range
        #   - Angular extents
        #   - Confidence

        # Prefilter
        detected_objects, object_ranges = self.prefilter(detected_objects)

        # Get LIDAR hitpoints with Actor ID associations
        hitpoints = self.__data_collector.get_carla_lidar_hitpoints()

        # Compute data needed for occlusion operation
        actor_angular_extents = self.compute_actor_angular_extents(detected_objects)
        detection_thresholds = self.compute_adjusted_detection_thresholds(detected_objects, object_ranges)

        # Geometry re-association
        # downsampled_hitpoints = self.sample_hitpoints(hitpoints,
        #                                               self.__simulated_sensor_config.geometry_reassociation.sample_count])
        # instantaneous_actor_id_association = self.compute_instantaneous_actor_id_association(downsampled_hitpoints,
        #                                                                                      detected_objects)
        # self.__actor_id_association = self.update_actor_id_association(instantaneous_actor_id_association,
        #                                                                self.__trailing_id_associations)
        # TODO detected_objects = self.reassociate(detected_objects, self.__actor_id_association)

        # Apply occlusion
        detected_objects = self.apply_occlusion(detected_objects, actor_angular_extents, hitpoints,
                                                detection_thresholds)
        # Apply noise
        detected_objects = self.apply_noise(detected_objects)

        return detected_objects

    # ------------------------------------------------------------------------------
    # CARLA Scene DetectedObject Retrieval
    # ------------------------------------------------------------------------------

    def get_scene_detected_objects(self):
        actors = self.__carla_world.get_actors()
        return [DetectedObject(self.__simulated_sensor_config, actor) for actor in actors]

    # ------------------------------------------------------------------------------
    # Prefilter
    # ------------------------------------------------------------------------------

    def prefilter(self, detected_objects):
        """
        Perform the following operations:
            1. Convert coordinates to sensor-centric frame.
            2. Compute range from sensor to object.
            3. Filter objects by allowed type.
            4. Filter objects beyond the configured range threshold.

        :param detected_objects:
        :return:
        """

        #

        # Filter by detected_object type
        # Actor.type_id and Actor.semantic_tags are available for determining type; semantic_tags effectively specifies the type of detected_object
        # Possible types are listed in the CARLA documentation: https://carla.readthedocs.io/en/0.9.10/ref_sensors/#semantic-segmentation-camera
        detected_objects = list(
            filter(lambda obj: obj.object_type in self.__simulated_sensor_config.prefilter.allowed_semantic_tags,
                   detected_objects))

        # Convert coordinates to sensor-centric frame
        detected_objects = [replace(obj, position=np.subtract(obj.position, self.__sensor.position)) for obj in
                            detected_objects]

        # Compute ranges
        object_ranges = dict([(obj.get_id(), np.linalg.norm(obj.position)) for obj in detected_objects])

        # Filter by radius
        detected_objects = list(
            filter(
                lambda obj: object_ranges[obj.get_id()] <= self.__simulated_sensor_config.prefilter.max_distance_meters,
                detected_objects))

        return detected_objects, object_ranges

    # ------------------------------------------------------------------------------
    # Computations
    # ------------------------------------------------------------------------------

    def compute_actor_angular_extents(self, detected_objects):
        return dict([(detected_object.id,
                      self.compute_actor_angular_extent(detected_object)) for detected_object in
                     detected_objects])

    def compute_actor_angular_extent(self, detected_object):
        bbox = detected_object.bbox
        corner_vec = np.array(bbox.extent)
        # Using combinatorics for conciseness
        all_corner_vectors = [np.matmul(np.diagflat(X), corner_vec) for X in itertools.product([-1, 1], repeat=3)]
        thetas = [self.compute_view_angle(v) for v in all_corner_vectors]
        return min(thetas), max(thetas)

    def compute_view_angle(self, vec):
        return np.arccos(np.vdot(self.__sensor.position, vec) / (np.linalg.norm(sensor.position) * np.linalg.norm(vec)))

    def compute_adjusted_detection_thresholds(self, config, detected_objects, object_ranges):
        return dict([(detected_object.id,
                      self.compute_adjusted_detection_threshold(config,
                                                                object_ranges[
                                                                    detected_object.get_id()], ))
                     for
                     detected_object in detected_objects])

    def compute_adjusted_detection_threshold(self, config, relative_object_position_vector):
        r = self.compute_range(relative_object_position_vector)
        dt_dr = config["detection_threshold_scaling_formula"][
            "hitpoint_detection_ratio_threshold_per_meter_change_rate"]
        t_nominal = config["detection_threshold_scaling_formula"]["nominal_hitpoint_detection_ratio_threshold"]
        # TODO Review this formula
        return dt_dr * r * t_nominal

    def compute_range(self, relative_object_position_vector):
        return np.linalg.norm(relative_object_position_vector)

    # return np.linalg.norm(detected_object["position"] - sensor["position"])

    # ------------------------------------------------------------------------------
    # Geometry Re-Association: Sampling
    # ------------------------------------------------------------------------------

    def sample_hitpoints(self, hitpoints, sample_size):
        """Randomly sample points inside each object's set of LIDAR hitpoints."""
        return dict([(obj_id, self.__rng.choice(object_hitpoints, sample_size)) for obj_id, object_hitpoints in
                     hitpoints.items()])

    # ------------------------------------------------------------------------------
    # Geometry Re-Association: Instantaneous Association
    # ------------------------------------------------------------------------------

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
        direct_nearest_neighbors = dict(
            [(obj_id, self.compute_closest_object_list(hitpoint_list, scene_objects)) for obj_id, hitpoint_list in
             downsampled_hitpoints.items()])

        # Vote within each dictionary key
        return dict([(obj_id, self.vote_closest_object(object_id_list)) for obj_id, object_id_list in
                     direct_nearest_neighbors.items()])

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

    # ------------------------------------------------------------------------------
    # Geometry Re-Association: Update Step
    # ------------------------------------------------------------------------------

    def update_actor_id_association(self, instantaneous_actor_id_association, trailing_id_associations):
        """
        Update the most recent association based on the current time step's instantaneously-derived association.
        """
        # TODO Should UKF be used?
        # For now the highest-voted id wins.

        # Extract all keys ("from" ID's) from all dictionaries
        combined = trailing_id_associations + instantaneous_actor_id_association
        all_keys = [association.keys() for association in combined]

        # Count number of each mapped ID reach from the from ID, and take the highest-voted
        updated_id_association = dict([(key, self.get_highest_counted_target_id(key, combined)) for key in all_keys])

        # Update trailing association queue
        self.__trailing_id_associations.appendleft(instantaneous_actor_id_association)

        return updated_id_association

    def get_highest_counted_target_id(self, key, combined):
        # Get all targets mapped from the key
        targets = [association.get(key) for association in combined]
        targets = filter(lambda x: x is not None, targets)

        # Count the number of times each target is mapped from the key
        counts = Counter(targets)

        # Return the target with the highest count
        return counts.most_common(1)[0][0]

    # ------------------------------------------------------------------------------
    # Occlusion Filter
    # ------------------------------------------------------------------------------

    def apply_occlusion(self, detected_objects, actor_angular_extents, sensor, hitpoints, detection_thresholds):
        return list(filter(
            lambda obj: self.is_visible(obj, actor_angular_extents[obj.id], sensor, hitpoints,
                                        detection_thresholds),
            detected_objects))

    def is_visible(self, detected_object, actor_angular_extent, sensor, hitpoints, detection_thresholds):
        # Compute threshold hitpoint count for this object
        num_expected_hitpoints = self.compute_expected_num_hitpoints(actor_angular_extent, sensor)
        detection_threshold_ratio = detection_thresholds[detected_object.id]
        min_hitpoint_count = detection_threshold_ratio * num_expected_hitpoints

        # Compare hitpoint count
        num_hitpoints = len(hitpoints[detected_object.id])

        return num_hitpoints >= min_hitpoint_count

    def compute_expected_num_hitpoints(self, actor_angular_extent, sensor):
        num_points_per_scan = sensor.points_per_second / sensor.rotation_frequency
        theta_resolution = sensor.fov_angular_width / num_points_per_scan
        return (actor_angular_extent[1] - actor_angular_extent[0]) / theta_resolution

    # ------------------------------------------------------------------------------
    # Noise Filter
    # ------------------------------------------------------------------------------

    def apply_noise(self, detected_objects, noise_model):
        detected_objects = noise_model.apply_position_noise(detected_objects)
        detected_objects = noise_model.apply_orientation_noise(detected_objects)
        detected_objects = noise_model.apply_type_noise(detected_objects)
        detected_objects = noise_model.apply_list_inclusion_noise(detected_objects)
        return detected_objects
