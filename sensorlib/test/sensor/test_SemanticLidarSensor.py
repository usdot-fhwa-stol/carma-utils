# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import unittest
from dataclasses import replace
from unittest.mock import MagicMock

import carla
import numpy as np

from collector.SensorDataCollector import SensorDataCollector
from noise_models.GaussianNoiseModel import GaussianNoiseModel
from objects.CarlaSensor import CarlaSensorBuilder
from objects.DetectedObject import DetectedObjectBuilder
from sensor.SemanticLidarSensor import SemanticLidarSensor
from util.CarlaUtils import CarlaUtils
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestSemanticLidarSensor(unittest.TestCase):

    def setUp(self):
        # Build the sensor
        self.raw_carla_sensor = SimulatedSensorTestUtils.generate_carla_sensor()
        self.carla_sensor = CarlaSensorBuilder.build_sensor(self.raw_carla_sensor)
        self.carla_sensor = replace(self.carla_sensor, position=np.array([1.0, 1.0, 0.0]))

        # Configs
        self.infrastructure_id = 123
        self.sensor_id = 4
        self.simulated_sensor_config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        self.carla_sensor_config = SimulatedSensorTestUtils.generate_lidar_sensor_config()
        self.noise_model_config = SimulatedSensorTestUtils.generate_noise_model_config()

        # Build a SemanticLidarSensor instance to access functions under test
        self.carla_world = MagicMock()
        self.data_collector = SensorDataCollector(self.carla_world, self.raw_carla_sensor)
        self.noise_model = GaussianNoiseModel(self.noise_model_config)
        self.sensor = SemanticLidarSensor(self.infrastructure_id, self.sensor_id, self.simulated_sensor_config,
                                          self.carla_sensor_config, self.carla_world,
                                          self.carla_sensor, self.data_collector, self.noise_model)

    def test_get_infrastructure_id(self):
        assert self.sensor.get_infrastructure_id() == self.infrastructure_id

    def test_get_id(self):
        assert self.sensor.get_id() == self.sensor_id

    def test_compute_detected_objects(self):
        # Generate test data
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        detected_objects = [replace(obj, carla_actor=None)
                            for obj in detected_objects]
        object_ranges = dict({
            0: 10.0,
            1: 11.0,
            2: 12.0,
            3: 13.0,
            4: 14.0,
            5: 15.0
        })
        hitpoints = dict({
            0: [MagicMock(), MagicMock(), MagicMock()],
            1: [MagicMock(), MagicMock(), MagicMock()],
            2: [MagicMock(), MagicMock(), MagicMock()],
            3: [MagicMock(), MagicMock(), MagicMock()],
            4: [MagicMock(), MagicMock(), MagicMock()],
            5: [MagicMock(), MagicMock(), MagicMock()]
        })
        actor_angular_extents = dict({
            0: (0.0, 1.096),
            1: (0.0, 1.096),
            2: (0.0, 1.096),
            3: (0.0, 1.096),
            4: (0.0, 1.096),
            5: (0.0, 1.096)
        })
        detection_thresholds = dict({
            0: 0.5,
            1: 0.5,
            2: 0.5,
            3: 0.6,
            4: 0.7,
            5: 0.7
        })
        timestamp = 0

        # Mock internal functions
        self.sensor.get_scene_detected_objects = MagicMock(return_value=detected_objects)
        self.sensor.prefilter = MagicMock(return_value=(detected_objects, object_ranges))
        self.sensor._SemanticLidarSensor__data_collector.get_carla_lidar_hitpoints = MagicMock(
            return_value=(0, hitpoints))
        self.sensor.compute_actor_angular_extents = MagicMock(return_value=actor_angular_extents)
        self.sensor.compute_adjusted_detection_thresholds = MagicMock(return_value=detection_thresholds)
        self.sensor.sample_hitpoints = MagicMock(return_value=hitpoints)
        self.sensor.compute_instantaneous_actor_id_association = MagicMock(
            return_value={0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5})
        self.sensor.apply_occlusion = MagicMock(return_value=detected_objects)
        self.sensor.apply_noise = MagicMock(return_value=detected_objects)
        self.sensor.update_object_frame_and_timestamps = MagicMock(return_value=detected_objects)

        # Call and provide assertions
        result = self.sensor.compute_detected_objects()

        self.sensor.get_scene_detected_objects.assert_called_once()
        self.sensor.prefilter.assert_called_once_with(detected_objects)
        self.sensor._SemanticLidarSensor__data_collector.get_carla_lidar_hitpoints.assert_called_once()
        self.sensor.compute_actor_angular_extents.assert_called_once_with(detected_objects)
        self.sensor.compute_adjusted_detection_thresholds.assert_called_once_with(detected_objects, object_ranges)
        self.sensor.apply_occlusion.assert_called_once_with(detected_objects, actor_angular_extents, hitpoints,
                                                            detection_thresholds)
        self.sensor.apply_noise.assert_called_once_with(detected_objects)
        self.sensor.update_object_frame_and_timestamps.assert_called_once_with(detected_objects, hitpoints, timestamp)

        self.assertEqual(result, detected_objects)
        self.assertEqual(self.sensor._SemanticLidarSensor__detected_objects, detected_objects)

        # Ensure cache is populated
        self.assertEqual(self.sensor.get_detected_objects(), detected_objects)

    def test_get_detected_objects(self):
        detected_objects = [MagicMock(id=3), MagicMock(id=4)]
        self.sensor._SemanticLidarSensor__detected_objects = detected_objects
        assert detected_objects == self.sensor.get_detected_objects()

    def test_get_scene_detected_objects(self):
        actors = [MagicMock(id=0), MagicMock(id=1)]
        detected_object = MagicMock(id=0)
        self.carla_world.get_actors = MagicMock(return_value=actors)
        old_fcn = DetectedObjectBuilder.build_detected_object
        DetectedObjectBuilder.build_detected_object = MagicMock(return_value=detected_object)
        result = self.sensor.get_scene_detected_objects()
        self.assertEqual(len(result), len(actors))
        for i in range(len(result)):
            self.assertEqual(result[i].id, 0)

        # Restore old function
        DetectedObjectBuilder.build_detected_object = old_fcn

    def test_prefilter(self):
        # Test filtering by type
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()[0:3]
        detected_objects[2] = replace(detected_objects[2], object_type="Bridge")
        filtered_objects, object_ranges = self.sensor.prefilter(detected_objects)
        self.assertEqual(len(filtered_objects), 2)
        self.assertEqual(filtered_objects[0].object_type, "Vehicles")
        self.assertEqual(filtered_objects[1].object_type, "Pedestrians")
        self.assertEqual(object_ranges[0], 38.635709988013005)
        self.assertEqual(object_ranges[1], 38.635709988013005)

        # Forceably adjust configured filter distance and test filtering by distance
        self.sensor._SemanticLidarSensor__simulated_sensor_config["prefilter"]["max_distance_meters"] = 0.0001
        filtered_objects, object_ranges = self.sensor.prefilter(detected_objects)
        self.assertEqual(len(filtered_objects), 0)

    def test_compute_actor_angular_extents(self):
        self.sensor.compute_actor_angular_extent = MagicMock(return_value=(0.5, 1.0))
        extents = self.sensor.compute_actor_angular_extents([MagicMock(id=0)])
        assert extents[0] == (0.5, 1.0)

    def test_compute_actor_angular_extent(self):
        # Data and call
        vec1 = np.array([4.0, 2.0, 4.0])
        vec2 = np.array([2.0, 4.0, 2.0])
        detected_object = MagicMock(id=0,
                                    bounding_box_in_world_coordinate_frame=[
                                        vec1,
                                        vec2
                                    ])
        angular_extents = self.sensor.compute_actor_angular_extent(detected_object)

        # Adjust to sensor frame for comparison
        vec1 = vec1 - np.array([1.0, 1.0, 0.0])
        vec2 = vec2 - np.array([1.0, 1.0, 0.0])

        h1 = np.arctan(1.0 / 3.0)
        h2 = np.arctan(3.0 / 1.0)
        v1 = np.arcsin(4.0 / np.linalg.norm(vec1))
        v2 = np.arcsin(2.0 / np.linalg.norm(vec2))
        expected_angular_extents = (np.abs(h2 - h1), np.abs(v2 - v1))
        assert np.allclose(angular_extents, expected_angular_extents)

    def test_compute_horizontal_angular_offset(self):
        vec1 = np.array([4.0, 2.0, 0.0])
        vec2 = np.array([2.0, 4.0, 0.0])
        angle1 = self.sensor.compute_horizontal_angular_offset(vec1)
        angle2 = self.sensor.compute_horizontal_angular_offset(vec2)
        self.assertAlmostEqual(angle1, np.arctan(1.0 / 3.0))
        self.assertAlmostEqual(angle2, np.arctan(3.0 / 1.0))

    def test_compute_vertical_angular_offset(self):
        vec1 = np.array([2.0, 0.0, 4.0])
        vec2 = np.array([4.0, 0.0, 2.0])
        angle1 = self.sensor.compute_vertical_angular_offset(vec1)
        angle2 = self.sensor.compute_vertical_angular_offset(vec2)
        vec1 = vec1 - np.array([1.0, 1.0, 0.0])
        vec2 = vec2 - np.array([1.0, 1.0, 0.0])
        self.assertAlmostEqual(angle1, np.arcsin(vec1[2] / np.linalg.norm(vec1)))
        self.assertAlmostEqual(angle2, np.arcsin(vec2[2] / np.linalg.norm(vec2)))

    def test_compute_adjusted_detection_thresholds(self):
        # Mock internal functions
        detected_objects = [MagicMock(id=3)]
        object_ranges = {3: 100.0}
        self.sensor.compute_adjusted_detection_threshold = MagicMock(return_value=0.7)

        # Call and provide assertions
        result = self.sensor.compute_adjusted_detection_thresholds(detected_objects, object_ranges)
        self.sensor.compute_adjusted_detection_threshold.assert_called_once_with(object_ranges[3])
        assert result == {3: 0.7}

    def test_compute_adjusted_detection_threshold(self):
        # Set up scaling problem
        range = 100.0
        dt_dr = -0.001
        t_nominal = 0.7

        expected_threshold = dt_dr * range * t_nominal

        # Update configurations and run the function
        self.sensor._SemanticLidarSensor__simulated_sensor_config["detection_threshold_scaling_formula"][
            "hitpoint_detection_ratio_threshold_per_meter_change_rate"] = dt_dr
        self.sensor._SemanticLidarSensor__simulated_sensor_config["detection_threshold_scaling_formula"][
            "nominal_hitpoint_detection_ratio_threshold"] = t_nominal

        threshold = self.sensor.compute_adjusted_detection_threshold(range)

        # Compare
        assert expected_threshold == threshold

    def test_sample_hitpoints(self):
        # Test data
        points_list = [
            np.array([1.0, 1.0, 1.0]),
            np.array([2.0, 2.0, 2.0]),
            np.array([3.0, 3.0, 3.0]),
            np.array([4.0, 4.0, 4.0]),
            np.array([5.0, 5.0, 5.0]),
            np.array([6.0, 6.0, 6.0])
        ]
        hitpoints = {
            0: points_list,
            1: points_list
        }

        # Restore and verify real sampling returns the expected sample size
        sampled_hitpoints = self.sensor.sample_hitpoints(hitpoints, 4)
        assert len(sampled_hitpoints[0]) == 4
        assert len(sampled_hitpoints[1]) == 4
        sampled_hitpoints = self.sensor.sample_hitpoints(hitpoints, 5)
        assert len(sampled_hitpoints[0]) == 5
        assert len(sampled_hitpoints[1]) == 5

        # Verify sampling does not repeat points
        sampled_hitpoints = self.sensor.sample_hitpoints(hitpoints, 6)
        assert len(sampled_hitpoints[0]) == 6
        assert len(sampled_hitpoints[1]) == 6
        assert np.alltrue([points_list[i] in sampled_hitpoints[0] for i in range(0, 6)])
        assert np.alltrue([points_list[i] in sampled_hitpoints[1] for i in range(0, 6)])

    def test_compute_instantaneous_actor_id_association(self):
        # Generate test scenario with hitpoints clustered around the object positions
        pos1 = np.array([4.0, 2.0, 0.0])
        pos2 = np.array([2.0, 4.0, 0.0])
        generated_detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        scene_objects = [
            replace(generated_detected_objects[0], id=0, position=pos1),
            replace(generated_detected_objects[1], id=1, position=pos2)
        ]
        points_list_1 = [
            pos1 + np.array([0.0, 0.0, 0.0]),
            pos1 + np.array([0.1, 0.0, 0.0]),
            pos1 + np.array([0.0, 0.1, 0.0]),
            pos1 + np.array([0.1, 0.1, 0.0]),
            pos1 + np.array([0.2, 0.0, 0.0]),
            pos1 + np.array([0.0, 0.2, 0.0])
        ]
        points_list_2 = [
            pos2 + np.array([0.0, 0.0, 0.0]),
            pos2 + np.array([0.1, 0.0, 0.0]),
            pos2 + np.array([0.0, 0.1, 0.0]),
            pos2 + np.array([0.1, 0.1, 0.0]),
            pos2 + np.array([0.2, 0.0, 0.0]),
            pos2 + np.array([0.0, 0.2, 0.0])
        ]
        downsampled_hitpoints = {
            0: points_list_1,
            1: points_list_2
        }

        # No change to a correct association
        id_association = self.sensor.compute_instantaneous_actor_id_association(downsampled_hitpoints, scene_objects)
        assert len(id_association) == 2
        assert id_association[0] == 0
        assert id_association[1] == 1

        # Opposite association
        downsampled_hitpoints = {
            1: points_list_1,
            0: points_list_2
        }
        id_association = self.sensor.compute_instantaneous_actor_id_association(downsampled_hitpoints, scene_objects)
        assert len(id_association) == 2
        assert id_association[0] == 1
        assert id_association[1] == 0

        # Test detected_objects with object IDs not picked up in the scan
        pos3 = np.array([100.0, 100.0, 0.0])
        scene_objects.append(replace(generated_detected_objects[2], id=100, position=pos3))
        downsampled_hitpoints = {
            0: points_list_1,
            1: points_list_2
        }
        id_association = self.sensor.compute_instantaneous_actor_id_association(downsampled_hitpoints, scene_objects)
        assert len(id_association) == 2
        assert id_association[0] == 0
        assert id_association[1] == 1

        # Test objects picked up in the scan which are not known in the truth state
        points_list_3 = [
            pos3 + np.array([0.0, 0.0, 0.0]),
            pos3 + np.array([0.1, 0.0, 0.0]),
            pos3 + np.array([0.0, 0.1, 0.0]),
            pos3 + np.array([0.1, 0.1, 0.0]),
            pos3 + np.array([0.2, 0.0, 0.0]),
            pos3 + np.array([0.0, 0.2, 0.0])
        ]
        downsampled_hitpoints = {
            0: points_list_1,
            1: points_list_2,
            2: points_list_3
        }
        scene_objects = scene_objects[0:-1]
        id_association = self.sensor.compute_instantaneous_actor_id_association(downsampled_hitpoints, scene_objects)
        assert len(id_association) == 2
        assert id_association[0] == 0
        assert id_association[1] == 1

        # Add third object within range of third point scan
        scene_objects.append(replace(generated_detected_objects[2], id=100, position=pos3))
        downsampled_hitpoints = {
            0: points_list_1,
            1: points_list_2,
            2: points_list_3
        }
        id_association = self.sensor.compute_instantaneous_actor_id_association(downsampled_hitpoints, scene_objects)
        assert len(id_association) == 3
        assert id_association[0] == 0
        assert id_association[1] == 1
        assert id_association[2] == 100

        # Move third object away from range of point scan (both points and truth state exist but are not within
        # association range)
        scene_objects[2] = replace(scene_objects[2], position=np.array([1000.0, 1000.0, 0.0]))
        id_association = self.sensor.compute_instantaneous_actor_id_association(downsampled_hitpoints, scene_objects)
        assert len(id_association) == 2
        assert id_association[0] == 0
        assert id_association[1] == 1

    def test_compute_closest_object_id_list(self):
        # Build test data
        hitpoint_list = [MagicMock(id=0), MagicMock(id=1)]
        generated_detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        scene_objects = [
            replace(generated_detected_objects[0], id=0),
            replace(generated_detected_objects[1], id=1)
        ]
        geometry_association_max_dist_in_meters = 0.2

        # Mock internal function
        self.sensor.compute_closest_object_id = MagicMock(return_value=0)

        # Call
        self.sensor.compute_closest_object_id_list(hitpoint_list, scene_objects,
                                                   geometry_association_max_dist_in_meters)

        # Assert internal function called correctly
        assert self.sensor.compute_closest_object_id.call_count == 2
        self.sensor.compute_closest_object_id.assert_any_call(hitpoint_list[0], scene_objects,
                                                              geometry_association_max_dist_in_meters)
        self.sensor.compute_closest_object_id.assert_any_call(hitpoint_list[1], scene_objects,
                                                              geometry_association_max_dist_in_meters)

    def test_compute_closest_object_id(self):
        # Generate test scenario with hitpoints clustered around the object positions
        pos1 = np.array([4.0, 2.0, 0.0])
        pos2 = np.array([2.0, 4.0, 0.0])
        generated_detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        scene_objects = [
            replace(generated_detected_objects[0], id=0, position=pos1),
            replace(generated_detected_objects[1], id=1, position=pos2)
        ]
        points_list_1 = [
            pos1 + np.array([0.0, 0.0, 0.0]),
            pos1 + np.array([0.1, 0.0, 0.0]),
            pos1 + np.array([0.0, 0.1, 0.0]),
            pos1 + np.array([0.1, 0.1, 0.0]),
            pos1 + np.array([0.2, 0.0, 0.0]),
            pos1 + np.array([0.0, 0.2, 0.0])
        ]
        points_list_2 = [
            pos2 + np.array([0.0, 0.0, 0.0]),
            pos2 + np.array([0.1, 0.0, 0.0]),
            pos2 + np.array([0.0, 0.1, 0.0]),
            pos2 + np.array([0.1, 0.1, 0.0]),
            pos2 + np.array([0.2, 0.0, 0.0]),
            pos2 + np.array([0.0, 0.2, 0.0])
        ]
        downsampled_hitpoints = {
            0: points_list_1,
            1: points_list_2
        }
        geometry_association_max_dist_in_meters = 0.6

        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[0][0], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 0
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[0][1], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 0
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[0][2], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 0
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[0][3], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 0
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[0][4], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 0
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[0][5], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 0
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[1][0], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 1
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[1][1], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 1
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[1][2], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 1
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[1][3], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 1
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[1][4], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 1
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[1][5], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id == 1

        # Point out of range
        geometry_association_max_dist_in_meters = 0.001
        id = self.sensor.compute_closest_object_id(downsampled_hitpoints[0][1], scene_objects,
                                                   geometry_association_max_dist_in_meters)
        assert id is None

    def test_vote_most_frequent_id(self):
        # Same
        assert 1 == self.sensor.vote_most_frequent_id([1, 1, 1])

        # Odd voting
        assert 1 == self.sensor.vote_most_frequent_id([1, 2, 1])
        assert 1 == self.sensor.vote_most_frequent_id([1, 1, 2])
        assert 2 == self.sensor.vote_most_frequent_id([2, 2, 1])

        # Even voting
        assert 1 == self.sensor.vote_most_frequent_id([1, 1])
        assert 1 == self.sensor.vote_most_frequent_id([1, 2])
        assert 2 == self.sensor.vote_most_frequent_id([2, 1])

        # Bad data
        assert 1 == self.sensor.vote_most_frequent_id([1, None, 1])
        assert 1 == self.sensor.vote_most_frequent_id([None, None, 1, None])
        assert self.sensor.vote_most_frequent_id([None, None]) is None
        assert self.sensor.vote_most_frequent_id([]) is None
        assert self.sensor.vote_most_frequent_id(None) is None

    def test_apply_occlusion(self):
        # Specify inputs
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        actor_angular_extents = {
            0: (0.2, 0.1),
            1: (0.2, 0.1),
            2: (0.2, 0.1),
            3: (0.2, 0.1),
            4: (0.2, 0.1),
            5: (0.2, 0.1)
        }
        hitpoints = {0: [MagicMock(id=0, objtype="hitpoint")], 1: [MagicMock(id=1, objtype="hitpoint")],
                     2: [MagicMock(id=2, objtype="hitpoint")], 3: [MagicMock(id=3, objtype="hitpoint")],
                     4: [MagicMock(id=4, objtype="hitpoint")], 5: [MagicMock(id=5, objtype="hitpoint")]}
        detection_thresholds = {0: 0.5, 1: 0.5, 2: 0.5, 3: 0.6, 4: 0.7, 5: 0.7}

        # Mock internal calls
        self.sensor.is_visible = MagicMock(return_value=True)

        # Call function
        occluded_objects = self.sensor.apply_occlusion(detected_objects, actor_angular_extents, hitpoints,
                                                       detection_thresholds)

        # Make assertions
        assert len(occluded_objects) == 6
        assert self.sensor.is_visible.call_count == 6
        self.sensor.is_visible.assert_called_with(actor_angular_extents[5], hitpoints[5], detection_thresholds[5])

    def test_is_visible(self):
        # Problem setup
        num_horizontal_points_per_scan = 360
        num_vertical_points_per_scan = 60

        horizontal_fov = np.deg2rad(15)
        vertical_fov = np.deg2rad(10)

        sensor_horizontal_fov = np.deg2rad(360)
        sensor_vertical_fov = np.deg2rad(60)

        rotation_frequency = 1
        points_per_second = num_horizontal_points_per_scan * num_vertical_points_per_scan * rotation_frequency

        # Mock the carla sensor
        carla_sensor = MagicMock(points_per_second=points_per_second, rotation_frequency=rotation_frequency,
                                 horizontal_fov=sensor_horizontal_fov,
                                 vertical_fov=sensor_vertical_fov, number_of_channels=num_vertical_points_per_scan)
        self.sensor._SemanticLidarSensor__sensor = carla_sensor

        # Object dimensions
        actor_angular_extents = (horizontal_fov, vertical_fov)
        detection_threshold_ratio = 0.7
        expected_threshold_value = 105

        # Test case 1: Number of hitpoints is greater than the minimum required
        object_hitpoints = [MagicMock() for _ in range(10 * expected_threshold_value)]
        result = self.sensor.is_visible(actor_angular_extents, object_hitpoints, detection_threshold_ratio)
        self.assertTrue(result)

        # Test case 2: Number of hitpoints is equal to the minimum required
        object_hitpoints = [MagicMock() for _ in range(expected_threshold_value)]
        result = self.sensor.is_visible(actor_angular_extents, object_hitpoints, detection_threshold_ratio)
        self.assertTrue(result)

        # Test case 2: Number of hitpoints is less than the minimum required
        object_hitpoints = [MagicMock() for _ in range(int(0.5 * expected_threshold_value))]
        result = self.sensor.is_visible(actor_angular_extents, object_hitpoints, detection_threshold_ratio)
        self.assertFalse(result)

    def test_compute_expected_num_hitpoints(self):
        # Problem setup
        num_horizontal_points_per_scan = 360
        num_vertical_points_per_scan = 60

        horizontal_fov = np.deg2rad(15)
        vertical_fov = np.deg2rad(10)

        sensor_horizontal_fov = np.deg2rad(360)
        sensor_vertical_fov = np.deg2rad(60)

        rotation_frequency = 1
        points_per_second = num_horizontal_points_per_scan * num_vertical_points_per_scan * rotation_frequency

        # Mock the carla sensor
        carla_sensor = MagicMock(points_per_second=points_per_second, rotation_frequency=rotation_frequency,
                                 horizontal_fov=sensor_horizontal_fov,
                                 vertical_fov=sensor_vertical_fov, number_of_channels=num_vertical_points_per_scan)
        self.sensor._SemanticLidarSensor__sensor = carla_sensor

        expected_expected_num_hitpoints = 150

        # Run and assertions
        expected_num_hitpoints = self.sensor.compute_expected_num_hitpoints(horizontal_fov, vertical_fov)
        self.assertAlmostEqual(expected_expected_num_hitpoints, expected_num_hitpoints)

    def test_apply_noise(self):
        detected_objects = [MagicMock(), MagicMock()]
        noise_model = MagicMock()

        # Mock the noise_model functions
        noise_model.apply_position_noise = MagicMock(return_value=detected_objects)
        noise_model.apply_orientation_noise = MagicMock(return_value=detected_objects)
        noise_model.apply_type_noise = MagicMock(return_value=detected_objects)
        noise_model.apply_list_inclusion_noise = MagicMock(return_value=detected_objects)

        # Test the method
        self.sensor._SemanticLidarSensor__noise_model = noise_model
        self.sensor.apply_noise(detected_objects)

        # Verify the noise model functions were called
        noise_model.apply_position_noise.assert_called_once_with(detected_objects)
        noise_model.apply_orientation_noise.assert_called_once_with(detected_objects)
        noise_model.apply_type_noise.assert_called_once_with(detected_objects)
        noise_model.apply_list_inclusion_noise.assert_called_once_with(detected_objects)

    def test_update_object_frame_and_timestamps(self):
        original_type = "Vehicles"
        expected_type = "Bridge"

        hitpoints = dict(
            [(i, [MagicMock(object_tag=CarlaUtils.get_semantic_tag_id(expected_type))]) for i in range(0, 6)])
        timestamp = 3

        # Generate objects in original world frame
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        detected_objects = [
            replace(detected_objects[0], object_type=original_type, timestamp=0, position=np.array([10.0, 10.0, 0.0])),
            replace(detected_objects[1], object_type=original_type, timestamp=0, position=np.array([11.0, 9.0, 0.0])),
            replace(detected_objects[2], object_type=original_type, timestamp=0, position=np.array([14.0, 9.0, 0.0])),
            replace(detected_objects[3], object_type=original_type, timestamp=0, position=np.array([8.0, 10.0, 0.0])),
            replace(detected_objects[4], object_type=original_type, timestamp=0, position=np.array([8.0, 14.0, 0.0])),
            replace(detected_objects[5], object_type=original_type, timestamp=0, position=np.array([14.0, 14.0, 0.0]))
        ]

        # Execute
        new_detected_objects = self.sensor.update_object_frame_and_timestamps(detected_objects, hitpoints, timestamp)

        # Assert object types updated
        assert new_detected_objects[0].object_type == "Bridge"
        assert new_detected_objects[1].object_type == "Bridge"
        assert new_detected_objects[2].object_type == "Bridge"
        assert new_detected_objects[3].object_type == "Bridge"
        assert new_detected_objects[4].object_type == "Bridge"
        assert new_detected_objects[5].object_type == "Bridge"

        # Assert timestamps updated
        assert new_detected_objects[0].timestamp == timestamp
        assert new_detected_objects[1].timestamp == timestamp
        assert new_detected_objects[2].timestamp == timestamp
        assert new_detected_objects[3].timestamp == timestamp
        assert new_detected_objects[4].timestamp == timestamp
        assert new_detected_objects[5].timestamp == timestamp

        # Assert the objects have been translated to the sensor-centric frame
        assert np.allclose(new_detected_objects[0].position, np.array([9.0, 9.0, 0.0]))
        assert np.allclose(new_detected_objects[1].position, np.array([10.0, 8.0, 0.0]))
        assert np.allclose(new_detected_objects[2].position, np.array([13.0, 8.0, 0.0]))
        assert np.allclose(new_detected_objects[3].position, np.array([7.0, 9.0, 0.0]))
        assert np.allclose(new_detected_objects[4].position, np.array([7.0, 13.0, 0.0]))
        assert np.allclose(new_detected_objects[5].position, np.array([13.0, 13.0, 0.0]))

    def test_update_object_frame_and_timestamps_from_hitpoint(self):
        # Build mock objects
        expected_type = 15
        carla_actor = MagicMock()
        carla_actor.id = 0
        carla_actor.semantic_tags = [10]
        carla_actor.get_world_vertices = MagicMock(return_value=[carla.Location(1.0, 2.0, 3.0),
                                                                 carla.Location(4.0, 5.0, 6.0),
                                                                 carla.Location(7.0, 8.0, 9.0),
                                                                 carla.Location(10.0, 11.0, 12.0)])
        carla_actor.get_transform = MagicMock(return_value=MagicMock(rotation=carla.Rotation(30.0, 90.0, 45.0)))
        carla_actor.get_location = MagicMock(return_value=carla.Location(1.0, 2.0, 3.0))
        carla_actor.get_velocity = MagicMock(return_value=carla.Vector3D(4.0, 5.0, 6.0))
        carla_actor.get_angular_velocity = MagicMock(return_value=carla.Vector3D(7.0, 8.0, 9.0))

        detected_object = DetectedObjectBuilder.build_detected_object(carla_actor, ["Vehicles"])

        hitpoints = {0: [MagicMock(object_tag=expected_type)]}

        timestamp = 3

        # Call and provide assertions
        corrected_objects = self.sensor.update_object_frame_and_timestamps_from_hitpoint(detected_object,
                                                                             hitpoints.get(detected_object.id),
                                                                             timestamp)
        assert "Bridge" == corrected_objects.object_type
        assert timestamp == corrected_objects.timestamp
