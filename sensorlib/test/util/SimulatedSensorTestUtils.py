# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import csv
from dataclasses import replace
from unittest.mock import MagicMock

import carla

from objects.DetectedObject import DetectedObjectBuilder


class SimulatedSensorTestUtils:
    TOLERANCE = 1e-4

    @staticmethod
    def generate_simulated_sensor_config():
        return {
            "prefilter": {
                "allowed_semantic_tags": ["Vehicles", "Pedestrians"],
                "max_distance_meters": 42
            },
            "detection_threshold_scaling_formula": {
                "nominal_hitpoint_detection_ratio_threshold": 0.6,
                "hitpoint_detection_ratio_threshold_per_meter_change_rate": -0.0033,
                "adjustable_threshold_scaling_parameters": {
                    "dropoff_rate": 0.01
                }
            },
            "geometry_reassociation": {
                "sample_count": 3,
                "geometry_association_max_dist_in_meters": 2.0,
            },
            "use_sensor_centric_frame": True
        }

    @staticmethod
    def generate_lidar_sensor_config():
        return {
            "lower_fov": -80.0,
            "upper_fov": 30.0,
            "channels": 60,
            "range": 100.0,
            "rotation_period": 0.05,
            "points_per_second": 10000
        }

    @staticmethod
    def generate_noise_model_config():
        return {
            "noise_model_name": "GaussianNoiseModel",
            "std_deviations": {
                "position_in_meters": [0.8, 0.8, 0.8],
                "orientation_in_radians": [0.1, 0.1, 0.1],
            },
            "stages": {
                "position_noise": True,
                "orientation_noise": True,
                "type_noise": True,
                "list_inclusion_noise": True
            },
            "type_noise": {
                "allowed_semantic_tags": [
                    "Buildings",
                    "Fences",
                    "Ground",
                    "GuardRail",
                    "Pedestrians",
                    "Poles",
                    "RoadLines",
                    "Roads",
                    "Sidewalks",
                    "Sky",
                    "Terrain",
                    "TrafficLight",
                    "TrafficSigns",
                    "Vegetation",
                    "Vehicles",
                    "Walls",
                    "Water"
                ]
            }
        }

    @staticmethod
    def generate_carla_sensor():
        """
        Generate a mock sensor.lidar.ray_cast_semantic.
        :return:
        """
        carla_sensor = MagicMock()
        sensor_config = MagicMock()
        sensor_config.channels = 1
        sensor_config.range = 1000.0
        sensor_config.rotation_frequency = 10.0
        sensor_config.points_per_second = 10000
        sensor_config.upper_fov = 20
        sensor_config.lower_fov = -40
        sensor_config.position = carla.Location(10.0, 15.0, 7.0)
        rotation = carla.Rotation(0, 0, 0)
        sensor_config.transform = MagicMock(
            return_value=carla.Transform(carla.Location(10.0, 15.0, 7.0), rotation))

        return carla_sensor

    @staticmethod
    def generate_test_data_detected_objects():

        # Mock the carla.Actor class
        carla_actor = MagicMock()
        carla_actor.id = 0
        carla_actor.attributes = dict()
        carla_actor.is_alive = True
        carla_actor.parent = None
        carla_actor.type_id = "vehicle.ford.mustang"

        extent = carla.Vector3D(2.94838892768239, 1.69796758051459, 1.0)
        location = carla.Location(20, 34.6410161513775, 0.0)
        rotation = carla.Rotation(3.0, 1.4, 4.0)
        carla_actor.get_bounding_box = MagicMock(
            return_value=MagicMock(extent=extent, location=location, rotation=rotation))

        carla_actor.get_acceleration = MagicMock(return_value=carla.Vector3D(0.0, 0.0, 0.0))
        carla_actor.get_angular_velocity = MagicMock(return_value=carla.Vector3D(0.0, 0.0, 0.005))
        carla_actor.get_location = MagicMock(return_value=location)
        carla_actor.get_transform = MagicMock(
            return_value=carla.Transform(carla.Location(10.0, 15.0, 7.0), rotation))
        carla_actor.get_velocity = MagicMock(return_value=carla.Vector3D(100.0, 1.0, 0.0))
        carla_actor.get_world = MagicMock(return_value=carla.World)

        # Construct the DetectedObject
        simulated_sensor_config = SimulatedSensorTestUtils.generate_simulated_sensor_config()
        detected_object = DetectedObjectBuilder.build_detected_object(carla_actor, simulated_sensor_config["prefilter"][
            "allowed_semantic_tags"])

        # Construct additional DetectedObject by adjustment
        return [
            replace(detected_object, id=0, object_type="Vehicles"),
            replace(detected_object, id=1, object_type="Pedestrians"),
            replace(detected_object, id=2, object_type="Pedestrians"),
            replace(detected_object, id=3, object_type="Pedestrians"),
            replace(detected_object, id=4, object_type="Vehicles"),
            replace(detected_object, id=5, object_type="Vehicles")
        ]

    @staticmethod
    def generate_test_data_hitpoints(self):
        # Read raw data
        test_points = []
        with open("data/test_data_hitpoints.csv", "r") as file:
            reader = csv.reader(file)
            for row in reader:
                theta, x, y, z = map(float, row)
                test_points.append((theta, x, y, z))

        # Chunk into data collection chunks
        num_chunks = 3
        semantic_lidar_measurements = []
        for n in num_chunks:
            chunk = test_points[n * len(test_points) // num_chunks: (n + 1) * len(test_points) // num_chunks]
            semantic_lidar_measurements.append(SimulatedSensorTestUtils.generate_semantic_lidar_measurement(chunk))
        return semantic_lidar_measurements

    @staticmethod
    def generate_semantic_lidar_measurement(test_points_chunk):
        raw_data = []
        for point in test_points_chunk:
            location = carla.Location(x=point[1], y=point[2], z=point[3])
            semantic_lidar_measurement = carla.SemanticLidarMeasurement(point=location, object_idx=123)
            raw_data.append(semantic_lidar_measurement)

        # Assign one representative angle for this measurement. This is a very rough approximation of the real collection data.
        return carla.SemanticLidarMeasurement(channels=1, horizontal_angle=test_points_chunk[0][0], raw_data=raw_data)
