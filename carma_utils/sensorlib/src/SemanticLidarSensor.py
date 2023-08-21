# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import itertools
from collections import deque
from dataclasses import replace

import numpy as np
import yaml

from src.SimulatedSensor import SimulatedSensor
from src.objects.DetectedObject import DetectedObject, DetectedObjectBuilder
from collections import Counter


class SemanticLidarSensor(SimulatedSensor):
    """
    Wrapper for the CARLA Semantic Lidar Sensor, with additional post-processing logic to compute a list of
    detected objects in the scene.

    Data is reported as the carla.SemanticLidarMeasurement type through the callback registered to
    carla.Sensor.listen(self, callback):

    https://carla.readthedocs.io/en/latest/python_api/#carla.Sensor.listen
    """

    def __init__(self, simulated_sensor_config, carla_sensor_config, carla_world, sensor, data_collector, noise_model):
        """
        Constructor.

        :param simulated_sensor_config: Configuration dictionary containing parameters specific to the wrapped sensor logic.
        :param carla_sensor_config: Parameters relevant to the CARLA sensor configuration.
        :param carla_world: Reference to the CARLA world object.
        :param sensor: CarlaSensor object wrapping the CARLA sensor actor.
        :param data_collector: DataCollector object handling collection and caching of raw sensor data from the CARLA simulation.
        :param noise_model: Noise model available to be used for nosie application to the output data.
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
        trailing_id_associations_count = simulated_sensor_config["geometry_reassociation"][
            "trailing_id_associations_count"]
        self.__trailing_id_associations = deque([{}], maxlen=trailing_id_associations_count)
        self.__rng = np.random.default_rng()

    # ------------------------------------------------------------------------------
    # Operation
    # ------------------------------------------------------------------------------

    def get_detected_objects_in_frame(self):
        """
        Main function used to query the currently-detected objects. Upon calling, the latest raw data cache is
        retrieved and sent through the processing pipeline to produce a list of DetectedObject objects.

        :return: List of DetectedObject objects.
        """

        # Get detected_object truth states from simulation
        detected_objects = self.get_scene_detected_objects()

        # Build the following lookup structures for infomration related to the detection or relationship between Sensor and DetectedObject:
        #   - Range
        #   - Angular extents
        #   - Confidence

        # Prefilter
        detected_objects, object_ranges = self.prefilter(detected_objects)

        # Get LIDAR hitpoints with Actor ID associations
        timestamp, hitpoints = self.__data_collector.get_carla_lidar_hitpoints()

        # Compute data needed for occlusion operation
        actor_angular_extents = self.compute_actor_angular_extents(detected_objects)
        detection_thresholds = self.compute_adjusted_detection_thresholds(detected_objects, object_ranges)

        # Instantaneous geometry association
        downsampled_hitpoints = self.sample_hitpoints(hitpoints,
                                                      self.__simulated_sensor_config.geometry_reassociation.sample_count)
        instantaneous_actor_id_association = self.compute_instantaneous_actor_id_association(downsampled_hitpoints,
                                                                                             detected_objects)

        # Geometry re-association
        self.update_actor_id_association(instantaneous_actor_id_association, self.__trailing_id_associations)
        hitpoints = self.update_object_ids(hitpoints)

        # Apply occlusion
        detected_objects = self.apply_occlusion(detected_objects, actor_angular_extents, hitpoints,
                                                detection_thresholds)
        # Apply noise
        detected_objects = self.apply_noise(detected_objects)

        # Update object type, reference frame, and detection time
        detected_objects = self.update_object_metadata(detected_objects, hitpoints, timestamp)

        return detected_objects

    # ------------------------------------------------------------------------------
    # CARLA Scene DetectedObject Retrieval
    # ------------------------------------------------------------------------------

    def get_scene_detected_objects(self):
        """
        Retrieve the current objects in scene. This collection is considered the "truth state" as it contains the set
        of objects in the world, as well as their positions, orientations, and variosu other state data.

        :return: DetectedObject wrappers objects referring to the actors.
        """
        actors = self.__carla_world.get_actors()
        return [DetectedObjectBuilder.build_detected_object(actor,
                                                            self.__simulated_sensor_config["prefilter"][
                                                                "allowed_semantic_tags"])
                for actor in actors]

    # ------------------------------------------------------------------------------
    # Prefilter
    # ------------------------------------------------------------------------------

    def prefilter(self, detected_objects):
        """
        Filter the truth state for objects of the desired detection type, and within range of the prefilter distance.

        :param detected_objects: List of objects currently considered for detection.
        :return: Filtered set of objects, and associated object ranges in a lookup structure.
        """
        #

        # Filter by detected_object type Actor.type_id and Actor.semantic_tags are available for determining type;
        # semantic_tags effectively specifies the type of detected_object Possible types are listed in the CARLA
        # documentation: https://carla.readthedocs.io/en/0.9.10/ref_sensors/#semantic-segmentation-camera
        detected_objects = list(
            filter(lambda obj: obj.object_type in self.__simulated_sensor_config["prefilter"]["allowed_semantic_tags"],
                   detected_objects))

        # Compute ranges
        object_ranges = dict(
            [(obj.id, np.linalg.norm(obj.position - self.__sensor.position)) for obj in detected_objects])

        # Filter by radius
        detected_objects = list(
            filter(
                lambda obj: object_ranges[obj.id] <= self.__simulated_sensor_config["prefilter"]["max_distance_meters"],
                detected_objects))

        return detected_objects, object_ranges

    # ------------------------------------------------------------------------------
    # Computations
    # ------------------------------------------------------------------------------

    def compute_actor_angular_extents(self, detected_objects):
        """
        Compute horizontal and vertical angular FOV extent from sensor perspective for each detected object.

        :param detected_objects: List of objects currently considered for detection.
        :return: Dictionary mapping actor ID to tuple of (horizontal, vertical) angular extents.
        """
        return dict([(detected_object.id,
                      self.compute_actor_angular_extent(detected_object)) for detected_object in
                     detected_objects])

    def compute_actor_angular_extent(self, detected_object):
        """
        Compute the horizontal and vertical angular FOV extent from sensor perspective for the given object.

        :param detected_object: Object under consideration.
        :return: Tuple containing the maximal horizontal and
        vertical angular extents consumed by the object, from the sensor's perspective.
        """
        corner_vectors_in_world_frame = detected_object.bounding_box_in_world_coordinate_frame
        theta = [self.compute_horizontal_angular_offset(vec) for vec in corner_vectors_in_world_frame]
        phi = [self.compute_vertical_angular_offset(vec) for vec in corner_vectors_in_world_frame]
        horizontal_fov = max(theta) - min(theta)
        vertical_fov = max(phi) - min(phi)
        return horizontal_fov, vertical_fov

    def compute_horizontal_angular_offset(self, vec):
        """Compute horizontal angle of a vector in relation to the sensor, as measured from the x axis."""
        p = vec - self.__sensor.position  # Position vector relative to sensor
        return np.arctan2(p[1], p[0])

    def compute_vertical_angular_offset(self, vec):
        """Compute the vertical angle of a vector in relation to the sensor, as measured from the x-y plane."""
        p = vec - self.__sensor.position
        return np.arcsin(p[2] / np.linalg.norm(p))

    def compute_adjusted_detection_thresholds(self, detected_objects, object_ranges):
        """
        Compute the adjusted detection threshold for each object, based on the object's range. This threshold varies
        with distance to accommodate ray spreading which naturally leads to lower hitpoint counts, risking object
        misdetections.
        """
        return dict([(detected_object.id,
                      self.compute_adjusted_detection_threshold(object_ranges[detected_object.id]))
                     for
                     detected_object in detected_objects])

    def compute_adjusted_detection_threshold(self, range):
        """Compute the detection threshold appropriate for the given range."""
        dt_dr = self.__simulated_sensor_config["detection_threshold_scaling_formula"][
            "hitpoint_detection_ratio_threshold_per_meter_change_rate"]
        t_nominal = self.__simulated_sensor_config["detection_threshold_scaling_formula"][
            "nominal_hitpoint_detection_ratio_threshold"]

        return dt_dr * range * t_nominal

    # ------------------------------------------------------------------------------
    # Geometry Re-Association: Sampling
    # ------------------------------------------------------------------------------

    def sample_hitpoints(self, hitpoints, sample_size):
        """Randomly sample points inside each object's set of LIDAR hitpoints. This is done to reduce size of the
        data being sent through the distance computation."""
        return dict([(obj_id, self.__rng.choice(object_hitpoints, sample_size)) for obj_id, object_hitpoints in
                     hitpoints.items()])

    # ------------------------------------------------------------------------------
    # Geometry Re-Association: Instantaneous Association
    # ------------------------------------------------------------------------------

    def compute_instantaneous_actor_id_association(self, hitpoints, scene_objects):
        """
        Compute the association of each sampled hitpoint to each actor, based on distance of the hitpoint to the
        actor's position.

        :param hitpoints: Dictionary mapping actor ID to list of hitpoints.
        :param scene_objects: List of objects currently considered for detection.
        return: Actor ID association applicable to this time step based on geometry-based association algorithm.
        """

        # Compute nearest neighbor for each hitpoint
        direct_nearest_neighbors = dict(
            [(obj_id, self.compute_closest_object_list(hitpoint_list, scene_objects,
                                                       self.__simulated_sensor_config["geometry_reassociation"][
                                                           "geometry_association_max_distance_threshold"])) for
             obj_id, hitpoint_list in
             hitpoints.items()])

        # Vote within each dictionary key
        return dict([(obj_id, self.vote_closest_object(object_list)) for obj_id, object_list in
                     direct_nearest_neighbors.items()])

    def compute_closest_object_list(self, hitpoints, scene_objects, geometry_association_max_distance_threshold):
        """Get the closest objects to each hitpoint."""
        return [self.compute_closest_object(hitpoint, scene_objects, geometry_association_max_distance_threshold) for
                hitpoint in hitpoints]

    def compute_closest_object(self, hitpoint, scene_objects, geometry_association_max_distance_threshold):
        """Compute the closest object to this hitpoint."""
        import numpy as np
        from scipy.spatial import distance
        object_positions = [obj.position for obj in scene_objects]
        distances = distance.cdist([hitpoint], object_positions)[0]
        closest_index = np.argmin(distances)

        # Observe a maximum object distance to preclude association with far-away objects
        if distances[closest_index] <= geometry_association_max_distance_threshold:
            closest_object = scene_objects[closest_index]
            return closest_object
        else:
            return None

    def vote_closest_object(self, object_list):
        """Determine the object with the highest number of votes as determined by the nearest-neighbor search."""
        return Counter([obj.id for obj in object_list]).most_common(1)[0][0]

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
        """Get the target ID with the highest count for the given key."""

        # Get all targets mapped from the key
        targets = [association.get(key) for association in combined]
        targets = filter(lambda x: x is not None, targets)

        # Count the number of times each target is mapped from the key
        counts = Counter(targets)

        # Return the target with the highest count
        return counts.most_common(1)[0][0]

    def update_object_ids(self, hitpoints):
        """Update object ID using the latest ID association"""
        return dict([(self.self.__actor_id_association[id], hitpoint_list) for id, hitpoint_list in hitpoints])

    # ------------------------------------------------------------------------------
    # Occlusion Filter
    # ------------------------------------------------------------------------------

    def apply_occlusion(self, detected_objects, actor_angular_extents, hitpoints, detection_thresholds):
        """
        Filter detected objects by occlusion. This is determined by comparing the number of hitpoints expected to hit
        the object as computed from the problem geometry, with the number of hitpoints detected.

        :param detected_objects: List of objects currently considered for detection.
        :param actor_angular_extents: Dictionary containing angular extent pairs referenced by object ID.
        :param hitpoints: Dictionary containing a list of hitpoints associated with each object ID.
        :param detection_thresholds: Thresholds computed for each object based on distance from the sensor.
        :return: List of objects filtered by occlusion.
        """
        return list(filter(
            lambda obj: self.is_visible(actor_angular_extents.get(obj.id), hitpoints.get(obj.id),
                                        detection_thresholds.get(obj.id)),
            detected_objects))

    def is_visible(self, actor_angular_extents, object_hitpoints, detection_threshold_ratio):
        """
        Compute if an object is visible based on the ratio of actual to expected hitpoints.

        :param actor_angular_extents: Tuple containing the horizontal and vertical angular extents of the object.
        :param object_hitpoints: List of hitpoints associated with the object.
        :param detection_threshold_ratio: Threshold ratio computed for this object based on distance from the sensor.
        :return: True if the object is visible, False otherwise.
        """

        if actor_angular_extents is None or object_hitpoints is None or detection_threshold_ratio is None:
            return False

        # TODO Review vertical component for computation
        horizontal_fov = actor_angular_extents[0]
        vertical_fov = actor_angular_extents[1]

        # Compute threshold hitpoint count for this object
        num_expected_hitpoints = self.compute_expected_num_horizontal_hitpoints(horizontal_fov)
        min_hitpoint_count = detection_threshold_ratio * num_expected_hitpoints

        # Compare hitpoint count
        num_hitpoints = len(object_hitpoints)

        return num_hitpoints >= min_hitpoint_count

    def compute_expected_num_horizontal_hitpoints(self, fov):
        """
        Compute the expected number of hitpoints for the given field of view. This result is heavily determined by
        the CARLA sensor configuration.
        
        :param fov: Scalar unoriented field of view in radians.
        :return: Expected number of hitpoints in a horizontal scan across a fov-sized sensor rotation.
        """
        num_points_per_scan = self.__sensor.points_per_second / self.__sensor.rotation_frequency
        theta_resolution = self.__sensor.fov_angular_width / num_points_per_scan
        return fov / theta_resolution

    # ------------------------------------------------------------------------------
    # Noise Filter
    # ------------------------------------------------------------------------------

    def apply_noise(self, detected_objects):
        """

        :param detected_objects:
        :return:
        """
        detected_objects = self.__noise_model.apply_position_noise(detected_objects)
        detected_objects = self.__noise_model.apply_orientation_noise(detected_objects)
        detected_objects = self.__noise_model.apply_type_noise(detected_objects)
        detected_objects = self.__noise_model.apply_list_inclusion_noise(detected_objects)
        return detected_objects

    # ------------------------------------------------------------------------------
    # Post Processing
    # ------------------------------------------------------------------------------

    def update_object_metadata(self, detected_objects, hitpoints, timestamp):
        return [self.update_object_metadata_from_hitpoint(obj, hitpoints) for obj in detected_objects]

    def update_object_metadata_from_hitpoint(self, obj, hitpoints, timestamp):
        """Get the metadata from the first hitpoint associated with the object, per the CARLA API."""
        hitpoint_list = hitpoints.get(obj.id)
        first_hitpoint = hitpoint_list[0] if hitpoint_list is not None else None

        # Update object type to match that reported from the CARLA semantic LIDAR sensor
        new_object_type = obj.object_type
        if first_hitpoint is not None:
            new_object_type = first_hitpoint.object_tag

        # If enabled, convert coordinates to sensor-centric frame
        new_position = obj.position
        if self.__simulated_sensor_config["use_sensor_centric_frame"]:
            new_position = np.subtract(obj.position, self.__sensor.position)

        return replace(obj,
                       object_type=new_object_type,
                       timestamp=timestamp,
                       position=new_position
                       )
