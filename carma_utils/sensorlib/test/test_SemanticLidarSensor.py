# Copyright (C) 2021 LEIDOS.
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

from src.SemanticLidarSensor import SemanticLidarSensor
from src.collector.SensorDataCollector import SensorDataCollector
from src.noise_models.GaussianNoiseModel import GaussianNoiseModel
from src.objects.CarlaSensor import CarlaSensorBuilder
from src.objects.DetectedObject import DetectedObjectBuilder
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestSemanticLidarSensor(unittest.TestCase):

    def setUp(self):
        # Build the sensor
        self.raw_carla_sensor = SimulatedSensorTestUtils.generate_carla_sensor()
        self.carla_sensor = CarlaSensorBuilder.build_sensor(self.raw_carla_sensor)
        self.carla_sensor = replace(self.carla_sensor, position=np.array([1.0, 1.0, 0.0]))

        # Configs
        self.simulated_sensor_config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        self.carla_sensor_config = SimulatedSensorTestUtils.generate_lidar_sensor_config()
        self.noise_model_config = SimulatedSensorTestUtils.generate_noise_model_config()

        # Build a SemanticLidarSensor instance to access functions under test
        self.carla_world = MagicMock()
        self.data_collector = SensorDataCollector(self.carla_world, self.raw_carla_sensor)
        self.noise_model = GaussianNoiseModel(self.noise_model_config)
        self.sensor = SemanticLidarSensor(self.simulated_sensor_config, self.carla_sensor_config, self.carla_world,
                                          self.carla_sensor, self.data_collector, self.noise_model)

    def test_get_detected_objects_in_frame(self):

        # Generate test data
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
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

        # Mock internal functions
        self.sensor.get_scene_detected_objects = MagicMock(return_value=detected_objects)
        self.sensor.prefilter = MagicMock(return_value=(detected_objects, object_ranges))
        self.sensor._SemanticLidarSensor__data_collector.get_carla_lidar_hitpoints = MagicMock(return_value=hitpoints)
        self.sensor.compute_actor_angular_extents = MagicMock(return_value=actor_angular_extents)
        self.sensor.compute_adjusted_detection_thresholds = MagicMock(return_value=detection_thresholds)
        self.sensor.update_object_types = MagicMock(return_value=detected_objects)
        self.sensor.apply_occlusion = MagicMock(return_value=detected_objects)
        self.sensor.apply_noise = MagicMock(return_value=detected_objects)
        self.sensor.transform_to_sensor_frame = MagicMock(return_value=detected_objects)

        # Call and provide assertions
        result = self.sensor.get_detected_objects_in_frame()

        self.sensor.get_scene_detected_objects.assert_called_once()
        self.sensor.prefilter.assert_called_once_with(detected_objects)
        self.sensor._SemanticLidarSensor__data_collector.get_carla_lidar_hitpoints.assert_called_once()
        self.sensor.compute_actor_angular_extents.assert_called_once_with(detected_objects)
        self.sensor.compute_adjusted_detection_thresholds.assert_called_once_with(detected_objects, object_ranges)
        self.sensor.update_object_types.assert_called_once_with(detected_objects, hitpoints)
        self.sensor.apply_occlusion.assert_called_once_with(detected_objects, actor_angular_extents, hitpoints,
                                                            detection_thresholds)
        self.sensor.apply_noise.assert_called_once_with(detected_objects)
        self.sensor.transform_to_sensor_frame.assert_called_once_with(detected_objects)

        self.assertEqual(result, detected_objects)

    def test_get_scene_detected_objects(self):
        actors = [MagicMock()]
        self.sensor._SemanticLidarSensor__carla_world.get_actors = MagicMock(return_value=actors)
        detected_object = MagicMock(id=0)
        old_fcn = DetectedObjectBuilder.build_detected_object
        DetectedObjectBuilder.build_detected_object = MagicMock(return_value=detected_object)
        DetectedObjectBuilder.build_detected_object.isCalledWith(actors[0], ["Pedestrians", "Vehicles"])

        # Undo the mock to avoid side effects which cause other tests to fail
        DetectedObjectBuilder.build_detected_object = old_fcn

    def test_prefilter(self):

        # Test filtering by type
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()[0:3]
        detected_objects[2] = replace(detected_objects[2], object_type="Pole")
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
        detected_objects = [MagicMock(id=0)]
        object_ranges = {0: 100.0}
        self.sensor.compute_adjusted_detection_threshold = MagicMock(return_value=0.0)

        # Call and provide assertions
        result = self.sensor.compute_adjusted_detection_thresholds(detected_objects, object_ranges)
        self.assertCalledOnceWith(detected_objects[0], object_ranges[0])

    def test_compute_adjusted_detection_thresholds(self):
        result = self.sensor.compute_adjusted_detection_threshold(100.0)
        self.assertAlmostEqual(-0.0033 * 0.6 * 100.0, result)

    def test_sample_hitpoints(self):
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
        self.sensor._SemanticLidarSensor__rng = MagicMock(choice=MagicMock(return_value=points_list[0:3]))
        sampled_hitpoints = self.sensor.sample_hitpoints(hitpoints, 3)
        assert len(sampled_hitpoints[0]) == 3
        assert len(sampled_hitpoints[1]) == 3
        assert np.allclose(sampled_hitpoints[0][0], np.array([1.0, 1.0, 1.0]))
        assert np.allclose(sampled_hitpoints[1][0], np.array([1.0, 1.0, 1.0]))

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
        assert id_association[0] == 0
        assert id_association[1] == 1

        # Opposite association
        downsampled_hitpoints = {
            1: points_list_1,
            0: points_list_2
        }
        id_association = self.sensor.compute_instantaneous_actor_id_association(downsampled_hitpoints, scene_objects)
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

    def test_compute_closest_object_list(self):
        self.assertTrue(False)

    def test_compute_closest_object(self):
        self.assertTrue(False)

    def test_vote_closest_object(self):
        self.assertTrue(False)

    def test_update_actor_id_association(self):
        self.assertTrue(False)

    def test_get_highest_counted_target_id(self):
        # Build mock objects
        expected_id = 1
        detected_object = MagicMock(id=expected_id)
        hitpoints = {expected_id: [MagicMock(), MagicMock(), MagicMock()]}

        # Call and provide assertions
        actual_id = self.sensor.get_highest_counted_target_id(detected_object, hitpoints)
        self.assertEqual(expected_id, actual_id)

    def test_update_object_ids(self):
        self.assertTrue(False)

    def test_apply_occlusion(self):
        detected_object = MagicMock(id=1)
        actor_angular_extents = {1: (0.0, 1.096)}
        hitpoints = {1: []}
        detection_thresholds = {1: 0.5}

        # Mock internal function
        self.sensor.is_visible = MagicMock(return_value=detected_object)

        # Call and apply assertions
        self.sensor.apply_occlusion([detected_object], actor_angular_extents, hitpoints, detection_thresholds)

        self.sensor.is_visible.assert_called_with(actor_angular_extents[detected_object.id],
                                                  hitpoints[detected_object.id],
                                                  detection_thresholds[detected_object.id]
                                                  )

    def test_is_visible(self):
        carla_sensor = MagicMock(points_per_second=10000, rotation_frequency=10, fov_angular_width=1.096)
        self.sensor._SemanticLidarSensor__sensor = carla_sensor
        fov = 1.096
        detection_threshold_ratio = 0.5

        # Test case 1: Number of hitpoints is greater than or equal to the minimum required
        object_hitpoints = []
        for i in range(4000):
            object_hitpoints += [MagicMock()]
        result = self.sensor.is_visible((fov, fov), object_hitpoints, detection_threshold_ratio)
        self.assertTrue(result)

        # Test case 2: Number of hitpoints is less than the minimum required
        object_hitpoints = []
        for i in range(4):
            object_hitpoints += [MagicMock()]
        result = self.sensor.is_visible((fov, fov), object_hitpoints, detection_threshold_ratio)
        self.assertFalse(result)

    def test_compute_expected_horizontal_num_hitpoints(self):
        carla_sensor = MagicMock(points_per_second=10000, rotation_frequency=10, fov_angular_width=1.096)
        self.sensor._SemanticLidarSensor__sensor = carla_sensor

        fov = 1.096
        num_points_per_scan = carla_sensor.points_per_second / carla_sensor.rotation_frequency
        theta_resolution = carla_sensor.fov_angular_width / num_points_per_scan
        expected_result = fov / theta_resolution

        result = self.sensor.compute_expected_num_horizontal_hitpoints(fov)

        self.assertEqual(result, expected_result)

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

    def test_update_object_metadata(self):

        # Build mock objects
        expected_type = "CorrectedVehicle"
        carla_actor = MagicMock()
        carla_actor.id = 0
        carla_actor.semantic_tags = ["Vehicles"]
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

        # Call and provide assertions
        corrected_objects = self.sensor.update_object_types([detected_object], hitpoints)
        assert expected_type == corrected_objects[0].object_type

    def test_update_object_metadata_from_hitpoint(self):
        original_type = "Vehicles"
        expected_type = "CorrectedVehicle"

        detected_object = MagicMock(id=0, object_type=original_type)
        hitpoints = {0: [MagicMock(object_tag=expected_type)]}

        actual_type = self.sensor.get_object_type_from_hitpoint(detected_object, hitpoints)
        self.assertEqual(expected_type, actual_type)

        ########

        # Generate objects in original world frame
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        detected_objects = [
            replace(detected_objects[0], position=np.array([10.0, 10.0, 0.0])),
            replace(detected_objects[1], position=np.array([11.0, 9.0, 0.0])),
            replace(detected_objects[2], position=np.array([14.0, 9.0, 0.0])),
            replace(detected_objects[3], position=np.array([8.0, 10.0, 0.0])),
            replace(detected_objects[4], position=np.array([8.0, 14.0, 0.0])),
            replace(detected_objects[5], position=np.array([14.0, 14.0, 0.0]))
        ]

        # Execute
        new_detected_objects = self.sensor.transform_to_sensor_frame(detected_objects)

        # Assert the objects have been translated to the sensor-centric frame
        assert np.allclose(new_detected_objects[0].position, np.array([9.0, 9.0, 0.0]))
        assert np.allclose(new_detected_objects[1].position, np.array([10.0, 8.0, 0.0]))
        assert np.allclose(new_detected_objects[2].position, np.array([13.0, 8.0, 0.0]))
        assert np.allclose(new_detected_objects[3].position, np.array([7.0, 9.0, 0.0]))
        assert np.allclose(new_detected_objects[4].position, np.array([7.0, 13.0, 0.0]))
        assert np.allclose(new_detected_objects[5].position, np.array([13.0, 13.0, 0.0]))
