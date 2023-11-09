# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

from collections import Counter
from dataclasses import replace

import numpy as np
from scipy.spatial import distance

from objects.DetectedObject import DetectedObjectBuilder
from sensor.SimulatedSensor import SimulatedSensor
from util.CarlaUtils import CarlaUtils
from util.HistoricalMapper import HistoricalMapper




class SemanticLidarSensor(SimulatedSensor):
    """
    Wrapper for the CARLA Semantic Lidar Sensor, with additional post-processing logic to compute a list of
    detected objects in the scene.

    Data is reported as the carla.SemanticLidarMeasurement type through the callback registered to
    carla.Sensor.listen(self, callback):

    https://carla.readthedocs.io/en/latest/python_api/#carla.Sensor.listen
    """

    def __init__(self, infrastructure_id, sensor_id, simulated_sensor_config, carla_sensor_config, carla_world, sensor,
                 data_collector, noise_model):
        """
        Constructor.

        :param infrastructure_id: Infrastructure ID with which the sensor is associated.
        :param sensor_id: Sensor ID.
        :param simulated_sensor_config: Configuration dictionary containing parameters specific to the wrapped sensor logic.
        :param carla_sensor_config: Parameters relevant to the CARLA sensor configuration.
        :param carla_world: Reference to the CARLA world object.
        :param sensor: CarlaSensor object wrapping the CARLA sensor actor.
        :param data_collector: DataCollector object handling collection and caching of raw sensor data from the CARLA simulation.
        :param noise_model: Noise model available to be used for noise application to the output data.
        """

        # Configuration
        super().__init__(infrastructure_id, sensor_id)
        self.__simulated_sensor_config = simulated_sensor_config
        self.__carla_sensor_config = carla_sensor_config

        # CARLA connection
        self.__carla_world = carla_world

        # Internal objects
        self.__sensor = sensor
        self.__data_collector = data_collector
        self.__noise_model = noise_model

        # Structures to store reassociation information
        self.__actor_id_association = {}
        trailing_id_associations_count = simulated_sensor_config[
            "geometry_reassociation"]["trailing_id_associations_count"]
        self.__trailing_id_associations = HistoricalMapper(trailing_id_associations_count)
        self.__rng = np.random.default_rng()

        # Object cache
        self.__detected_objects = []

    # ------------------------------------------------------------------------------
    # Primary functions
    # ------------------------------------------------------------------------------

    def compute_detected_objects(self):
        """
        Main function used to query the currently-detected objects. Upon calling, the latest raw data cache is
        retrieved and sent through the processing pipeline to produce a list of DetectedObject objects.

        :return: List of DetectedObject objects serialized in JSON form.
        """

        # Get detected_object truth states from simulation
        detected_objects = self.get_scene_detected_objects()

        #check if there's object in range
        if not detected_objects:
            self.__detected_objects.clear()
            return []

        # Prefilter
        detected_objects, object_ranges = self.prefilter(detected_objects)

        #check if there's object in range
        if not detected_objects:
            self.__detected_objects.clear()
            return []

        # Get LIDAR hitpoints with Actor ID associations
        timestamp, hitpoints = self.__data_collector.get_carla_lidar_hitpoints()

        # Compute data needed for occlusion operation
        actor_angular_extents = self.compute_actor_angular_extents(detected_objects)
        detection_thresholds = self.compute_adjusted_detection_thresholds(detected_objects, object_ranges)

        # Instantaneous geometry association
        sample_size = self.__simulated_sensor_config["geometry_reassociation"]["sample_count"]
        downsampled_hitpoints = self.sample_hitpoints(hitpoints, sample_size)
        instantaneous_actor_id_association = self.compute_instantaneous_actor_id_association(downsampled_hitpoints,
                                                                                             detected_objects)

        # Geometry re-association
        self.update_actor_id_association(instantaneous_actor_id_association)
        hitpoints = self.update_hitpoint_ids_from_association(hitpoints)

        # Apply occlusion TODO!
        # https://usdot-carma.atlassian.net/browse/CDAR-435
        #detected_objects = self.apply_occlusion(detected_objects, actor_angular_extents, hitpoints,
        #                                        detection_thresholds)

        # Apply noise
        detected_objects = self.apply_noise(detected_objects)

        # Update object type, reference frame, and detection time
        detected_objects = self.update_object_metadata(detected_objects, hitpoints, timestamp)

        self.__detected_objects = detected_objects

        return detected_objects

    def get_detected_objects(self):
        """Returns the latest detected objects."""
        return self.__detected_objects

    # ------------------------------------------------------------------------------
    # CARLA Scene DetectedObject Retrieval
    # ------------------------------------------------------------------------------

    def get_scene_detected_objects(self):
        """
        Retrieve the current objects in scene. This collection is considered the "truth state" as it contains the set
        of objects in the world, as well as their positions, orientations, and various other state data.

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
            filter(lambda obj: obj is not None, detected_objects))

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
        """Down sample each object's hitpoint list

        Randomly sample points inside each object's set of LIDAR
        hitpoints. This is done to reduce size of the data being sent
        through the distance computation. If the hitpoint population of
        a specific object is less than the sample size, the whole
        hitpoint population will be used (i.e., the object's list of
        hitpoints will remain unchanged).

        :param hitpoints: lidar points associated with each object
        :param sample_size: maximum size of the object's new hitpoint list
        :return: downsampled hitpoints
        """
        return {
            # The choice() function will raise an error if we try to
            # sample more than the population
            id_: self.__rng.choice(points, min(len(points), sample_size), replace=False)
            for id_, points in hitpoints.items()
        }

    # ------------------------------------------------------------------------------
    # Geometry Re-Association: Instantaneous Association
    # ------------------------------------------------------------------------------

    def compute_instantaneous_actor_id_association(self, hitpoints, scene_objects):
        """
        Compute the association of each sampled hitpoint to each actor, based on distance of the hitpoint to the
        actor's position.

        :param hitpoints: Dictionary mapping actor ID to list of hitpoints.
        :param scene_objects: List of objects currently considered for detection.
        :return: Actor ID association applicable to this time step based on geometry-based association algorithm,
            in the form of a dictionary mapping {hitpoint ID -> actor ID}.
        """

        # Compute nearest neighbor for each hitpoint
        direct_nearest_neighbors = dict(
            [(hit_id, self.compute_closest_object_id_list(hitpoint_list, scene_objects,
                                                          self.__simulated_sensor_config["geometry_reassociation"][
                                                              "geometry_association_max_distance_threshold"])) for
             hit_id, hitpoint_list in
             hitpoints.items()])

        # Vote within each dictionary key
        id_association = [(hit_id, self.vote_most_frequent_id(object_id_list)) for hit_id, object_id_list in
                          direct_nearest_neighbors.items()]

        # Filter unassociated hitpoints
        return dict(filter(lambda x: x[1] is not None, id_association))

    def compute_closest_object_id_list(self, hitpoint_list, scene_objects, geometry_association_max_distance_threshold):
        """Get the closest objects to each hitpoint."""
        return [self.compute_closest_object_id(hitpoint, scene_objects, geometry_association_max_distance_threshold) for
                hitpoint in hitpoint_list]

    def compute_closest_object_id(self, hitpoint, scene_objects, geometry_association_max_distance_threshold):
        """
        Compute the closest object to this hitpoint, which lies within a maximum distance threshold.

        The threshold prevents association between a point and object which are very far apart.
        """

        object_positions = [obj.position for obj in scene_objects]
        distances_list = distance.cdist([hitpoint], object_positions)
        if len(distances_list) <= 0 or len(distances_list[0]) <= 0:
            return None
        distances = distances_list[0]
        closest_index = np.argmin(distances)

        # May result from bad or empty data
        if closest_index is None or closest_index < 0:
            return None

        # Observe a maximum object distance to preclude association with far-away objects
        if distances[closest_index] > geometry_association_max_distance_threshold:
            return None

        # Return closest object's ID
        closest_object = scene_objects[closest_index]
        return closest_object.id

    def vote_most_frequent_id(self, object_id_list):
        """Determine the object with the highest number of votes as determined by the nearest-neighbor search."""

        if object_id_list is None or len(object_id_list) == 0:
            return None

        # Do not consider incorrectly associated ID's
        object_id_list = list(filter(lambda x: x is not None, object_id_list))

        # Identify the most frequently occurring ID
        most_frequent_pair = Counter(object_id_list).most_common(1)
        if most_frequent_pair is not None and len(most_frequent_pair) > 0:
            return most_frequent_pair[0][0]
        else:
            return None

    # ------------------------------------------------------------------------------
    # Geometry Re-Association: Update Step
    # ------------------------------------------------------------------------------

    def update_actor_id_association(self, instantaneous_actor_id_association):
        """
        Update the most recent association based on the current time step's instantaneously-derived association.
        """

        if instantaneous_actor_id_association is None or len(instantaneous_actor_id_association) == 0:
            return self.__actor_id_association

        # Prepend instantaneous association to trailing associations
        # Possible enhancement issue: https://github.com/usdot-fhwa-stol/carma-platform/issues/2142

        for hitpoint_id, obj_id in instantaneous_actor_id_association.items():
            self.__trailing_id_associations.push(hitpoint_id, obj_id)

        # Recompute current association
        self.__actor_id_association = dict()
        for hitpoint_id in self.__trailing_id_associations.get_keys():
            q = list(self.__trailing_id_associations.get_queue(hitpoint_id))
            obj_id = self.vote_most_frequent_id(q)
            self.__actor_id_association[hitpoint_id] = obj_id

        return self.__actor_id_association

    def update_hitpoint_ids_from_association(self, hitpoints):
        """Update object ID using the latest ID association"""

        # Update to the current association's mapped ID, or use the original ID as default
        return dict(
            [(self.__actor_id_association.get(id, id), hitpoint_list) for id, hitpoint_list in hitpoints.items()])

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

        horizontal_fov = actor_angular_extents[0]
        vertical_fov = actor_angular_extents[1]

        # Compute threshold hitpoint count for this object
        num_expected_hitpoints = self.compute_expected_num_hitpoints(horizontal_fov, vertical_fov)
        min_hitpoint_count = detection_threshold_ratio * num_expected_hitpoints

        # Compare hitpoint count
        num_hitpoints = len(object_hitpoints)

        return num_hitpoints >= min_hitpoint_count

    def compute_expected_num_hitpoints(self, horizontal_fov, vertical_fov):
        """
        Compute the expected number of hitpoints for the given field of view. This result is heavily determined by
        the CARLA sensor configuration.

        :param horizontal_fov: Horizontal field of view in radians.
        :param vertical_fov: Vertical field of view in radians.
        :return: Expected number of hitpoints in a scan across the specified field of view.
        """
        num_horizontal_points_per_scan = (
                                                     self.__sensor.points_per_second / self.__sensor.rotation_frequency) / self.__sensor.number_of_channels
        horizontal_angular_resolution = self.__sensor.horizontal_fov / num_horizontal_points_per_scan

        num_vertical_points_per_scan = self.__sensor.number_of_channels
        vertical_angular_resolution = self.__sensor.vertical_fov / num_vertical_points_per_scan

        return (horizontal_fov / horizontal_angular_resolution) * (vertical_fov / vertical_angular_resolution)

    # ------------------------------------------------------------------------------
    # Noise Filter
    # ------------------------------------------------------------------------------

    def apply_noise(self, detected_objects):
        """
        Apply noise to the detected objects.

        :param detected_objects: List of objects currently considered for detection.
        :return: Objects with noise applied.
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
        """
        Update object metadata including object type, detection timestamp in seconds, and adjusting the coordinates to the
        sensor-centric frame.

        :param detected_objects: List of objects currently considered for detection.
        :param hitpoints: Dictionary containing a list of hitpoints associated with each object ID.
        :param timestamp: Timestamp of the current frame (seconds).
        :return: List of objects with updated metadata.
        """
        return [self.update_object_metadata_from_hitpoint(obj, hitpoints.get(obj.id), timestamp)
                for obj in detected_objects]

    def update_object_metadata_from_hitpoint(self, obj, hitpoints, timestamp):
        """
        Update the object metadata (object type, timestamp, coordinates).

        :param obj: Detected object.
        :param hitpoints: Hitpoints associated with the object.
        :param timestamp: Timestamp of the current frame (seconds).
        :return: Updated object.
        """

        # Get the metadata from the first hitpoint associated with the object, per the CARLA API.
        hitpoint_list = hitpoints
        first_hitpoint = hitpoint_list[0] if hitpoint_list is not None else None

        # Update object type to match that reported from the CARLA semantic LIDAR sensor
        new_object_type = obj.object_type
        if first_hitpoint is not None:
            new_object_type = CarlaUtils.get_semantic_tag_name(first_hitpoint.object_tag)

        # If enabled, convert coordinates to sensor-centric frame
        new_position = obj.position
        if self.__simulated_sensor_config["use_sensor_centric_frame"]:
            new_position = np.subtract(obj.position, self.__sensor.position)

        return replace(obj,
                       object_type=new_object_type,
                       timestamp=timestamp,
                       position=new_position
                       )
