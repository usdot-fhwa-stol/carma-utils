import itertools
from collections import deque
from dataclasses import replace

import numpy as np
import yaml

from src.SimulatedSensor import SimulatedSensor
from src.objects.DetectedObject import DetectedObject, DetectedObjectBuilder
from collections import Counter


class SemanticLidarSensor(SimulatedSensor):

    def __init__(self, simulated_sensor_config, carla_sensor_config, carla_world, sensor, data_collector, noise_model):
        """
        :param simulated_sensor_config:
        :param carla_sensor_config:
        :param carla_world:
        :param sensor: CarlaSensor object wrapping the CARLA sensor actor.
        :param data_collector:
        :param noise_model:
        """
        self.__simulated_sensor_config = simulated_sensor_config
        self.__carla_sensor_config = carla_sensor_config

        # Fundamental internal objects
        self.__carla_world = carla_world
        self.__sensor = sensor
        self.__data_collector = data_collector
        self.__noise_model = noise_model

        # Structures to store reassociation information
        self.__actor_id_association = {}
        trailing_id_associations_count = simulated_sensor_config["geometry_reassociation"]["trailing_id_associations_count"]
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
        # self.update_actor_id_association(instantaneous_actor_id_association,
        #                                                                self.__trailing_id_associations)

        # Update object IDs to match the association
        hitpoints = self.update_object_ids(hitpoints)

        # Update actor types to match that reported from the CARLA semantic LIDAR sensor
        detected_objects = self.update_object_types(detected_objects, hitpoints)

        # Apply occlusion
        detected_objects = self.apply_occlusion(detected_objects, actor_angular_extents, hitpoints,
                                                detection_thresholds)
        # Apply noise
        detected_objects = self.apply_noise(detected_objects)

        # Transform object locations to sensor frame
        detected_objects = self.transform_to_sensor_frame(detected_objects)

        return detected_objects

    # ------------------------------------------------------------------------------
    # CARLA Scene DetectedObject Retrieval
    # ------------------------------------------------------------------------------

    def get_scene_detected_objects(self):
        actors = self.__carla_world.get_actors()
        return [DetectedObjectBuilder.build_detected_object(actor,
                                                            self.__simulated_sensor_config.prefilter.allowed_semantic_tags)
                for actor in actors]

    # ------------------------------------------------------------------------------
    # Prefilter
    # ------------------------------------------------------------------------------

    def prefilter(self, detected_objects):
        """
        Perform the following operations:
            1. Compute range from sensor to object.
            2. Filter objects by allowed type.
            3. Filter objects beyond the configured range threshold.

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

        # Compute ranges
        object_ranges = dict(
            [(obj.get_id(), np.linalg.norm(obj.position - self.__sensor.position)) for obj in detected_objects])

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
        """
        Compute horizontal and vertical angular FOV extent from sensor perspective for each detected object.
        param: detected_objects: List of DetectedObject objects.
        return: Dictionary of actor ID to tuple of (horizontal, vertical) angular extents.
        """
        return dict([(detected_object.id,
                      self.compute_actor_angular_extent(detected_object)) for detected_object in
                     detected_objects])

    def compute_actor_angular_extent(self, detected_object):
        """
        """
        corner_vectors_in_world_frame = detected_object.bounding_box_in_world_coordinate_frame
        theta = [self.compute_horizontal_angular_extent(vec) for vec in corner_vectors_in_world_frame]
        phi = [self.compute_vertical_angular_extent(vec) for vec in corner_vectors_in_world_frame]
        horizontal_fov = max(theta) - min(theta)
        vertical_fov = max(phi) - min(phi)
        return horizontal_fov, vertical_fov

    def compute_horizontal_angular_extent(self, vec):
        p = vec - self.__sensor.position  # Position vector relative to sensor
        return np.arctan2(p[1], p[0])

    def compute_vertical_angular_extent(self, vec):
        p = vec - self.__sensor.position
        return np.arcsin(p[2], np.linalg.norm(p))

    def compute_adjusted_detection_thresholds(self, detected_objects, object_ranges):
        return dict([(detected_object.id,
                      self.compute_adjusted_detection_threshold(object_ranges[detected_object.get_id()]))
                     for
                     detected_object in detected_objects])

    def compute_adjusted_detection_threshold(self, range):
        dt_dr = self.__simulated_sensor_config["detection_threshold_scaling_formula"][
            "hitpoint_detection_ratio_threshold_per_meter_change_rate"]
        t_nominal = self.__simulated_sensor_config["detection_threshold_scaling_formula"][
            "nominal_hitpoint_detection_ratio_threshold"]
        # TODO Review this formula
        return dt_dr * range * t_nominal

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
        self.__actor_id_association = dict(
            [(key, self.get_highest_counted_target_id(key, combined)) for key in all_keys])

        # Update trailing association queue
        self.__trailing_id_associations.appendleft(instantaneous_actor_id_association)

    def get_highest_counted_target_id(self, key, combined):
        # Get all targets mapped from the key
        targets = [association.get(key) for association in combined]
        targets = filter(lambda x: x is not None, targets)

        # Count the number of times each target is mapped from the key
        counts = Counter(targets)

        # Return the target with the highest count
        return counts.most_common(1)[0][0]

    # ------------------------------------------------------------------------------
    # Update Object IDs and Types
    # ------------------------------------------------------------------------------

    def update_object_ids(self, hitpoints):
        return dict([(self.self.__actor_id_association[id], hitpoint_list) for id, hitpoint_list in hitpoints])

    def update_object_types(self, detected_objects, hitpoints):
        return [replace(obj, object_type=self.get_object_type_from_hitpoint(obj, hitpoints)) for obj in
                detected_objects]

    def get_object_type_from_hitpoint(self, detected_object, hitpoints):
        """Get the object type from the first hitpoint associated with the object, per the CARLA API."""
        hitpoint_list = hitpoints.get(detected_object.id)
        if hitpoint_list is not None:
            first_hitpoint = hitpoint_list.get(0)
            if first_hitpoint is not None:
                return first_hitpoint.object_tag

        return detected_object.object_type

    # ------------------------------------------------------------------------------
    # Occlusion Filter
    # ------------------------------------------------------------------------------

    def apply_occlusion(self, detected_objects, actor_angular_extents, hitpoints, detection_thresholds):
        return list(filter(
            lambda obj: self.is_visible(obj, actor_angular_extents.get(obj.id), hitpoints.get(obj.id),
                                        detection_thresholds.get(obj.id))),
            detected_objects)

    def is_visible(self, detected_object, actor_angular_extents, object_hitpoints, detection_threshold_ratio):

        if actor_angular_extents is None or object_hitpoints is None or detection_threshold_ratio is None:
            return False

        # TODO Review vertical component for computation
        horizontal_fov = actor_angular_extents[0]
        vertical_fov = actor_angular_extents[1]

        # Compute threshold hitpoint count for this object
        num_expected_hitpoints = self.compute_expected_num_hitpoints(horizontal_fov)
        min_hitpoint_count = detection_threshold_ratio * num_expected_hitpoints

        # Compare hitpoint count
        num_hitpoints = len(object_hitpoints)

        return num_hitpoints >= min_hitpoint_count

    def compute_expected_num_hitpoints(self, fov):
        num_points_per_scan = self.__sensor.points_per_second / self.__sensor.rotation_frequency
        theta_resolution = self.__sensor.fov_angular_width / num_points_per_scan
        return fov / theta_resolution

    # ------------------------------------------------------------------------------
    # Noise Filter
    # ------------------------------------------------------------------------------

    def apply_noise(self, detected_objects):
        detected_objects = self.__noise_model.apply_position_noise(detected_objects)
        detected_objects = self.__noise_model.apply_orientation_noise(detected_objects)
        detected_objects = self.__noise_model.apply_type_noise(detected_objects)
        detected_objects = self.__noise_model.apply_list_inclusion_noise(detected_objects)
        return detected_objects

    # ------------------------------------------------------------------------------
    # Post Processing
    # ------------------------------------------------------------------------------

    def transform_to_sensor_frame(self, detected_objects):
        """Convert coordinates to sensor-centric frame"""
        return [replace(obj, position=np.subtract(obj.position, self.__sensor.position)) for obj in
                detected_objects]
