# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import os
import unittest
from abc import abstractmethod
import numpy as np
import open3d as o3d
import time
from datetime import datetime

import carla

from sensorlib.carla_cda_sim_api import CarlaCDASimAPI
from tests.integration.integration_test_utilities import IntegrationTestUtilities
from util.carla_utils import CarlaUtils


class SensorlibIntegrationTestRunner(unittest.TestCase):
    LABEL_COLORS = np.array([
        (255, 255, 255),  # None
        (70, 70, 70),  # Building
        (100, 40, 40),  # Fences
        (55, 90, 80),  # Other
        (220, 20, 60),  # Pedestrian
        (153, 153, 153),  # Pole
        (157, 234, 50),  # RoadLines
        (128, 64, 128),  # Road
        (244, 35, 232),  # Sidewalk
        (107, 142, 35),  # Vegetation
        (0, 0, 142),  # Vehicle
        (102, 102, 156),  # Wall
        (220, 220, 0),  # TrafficSign
        (70, 130, 180),  # Sky
        (81, 0, 81),  # Ground
        (150, 100, 100),  # Bridge
        (230, 150, 140),  # RailTrack
        (180, 165, 180),  # GuardRail
        (250, 170, 30),  # TrafficLight
        (110, 190, 160),  # Static
        (170, 120, 50),  # Dynamic
        (45, 60, 150),  # Water
        (145, 170, 100),  # Terrain
    ]) / 255.0  # normalize each channel [0-1] since is what Open3D uses

    def setUp(self):
        # Connect to CARLA
        carla_host = os.getenv('CARLA_HOST', 'localhost')
        carla_port = int(os.getenv('CARLA_PORT', '2000'))
        self.carla_world = SensorlibIntegrationTestRunner.get_carla_connection(carla_host, carla_port)
        self.api = SensorlibIntegrationTestRunner.build_api_object(self.carla_world)

        # Remove previous integration test artifacts
        IntegrationTestUtilities.delete_simulation_objects(self.carla_world)

    @staticmethod
    def get_carla_connection(carla_host, carla_port):
        client = carla.Client(carla_host, carla_port)
        return client.get_world()

    @staticmethod
    def build_api_object(carla_world):
        return CarlaCDASimAPI.build_from_world(carla_world)

    def launch_display_windows(self, sensor, sensor_position, carla_sensor_config, point_list):

        # Enable LIDAR visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name="Carla Lidar",
            width=960,
            height=540,
            left=480,
            top=270)
        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 1
        vis.get_render_option().show_coordinate_frame = True

        self.add_open3d_axis(vis)

        frame = 0
        while True:
            if frame == 2:
                vis.add_geometry(point_list)
            vis.update_geometry(point_list)

            vis.poll_events()
            vis.update_renderer()

            if frame % 120 == 0:
                detected_objects = sensor.get_detected_objects()
                print(f"Detected objects: {len(detected_objects)} objects")
                for detected_object in detected_objects:
                    print(f"ID: {detected_objects.get_id()}")

            # This can fix Open3D jittering issues:
            time.sleep(0.005)
            self.carla_world.tick()
            frame += 1

    def add_open3d_axis(self, vis):
        """Add a small 3D axis on Open3D Visualizer"""
        axis = o3d.geometry.LineSet()
        axis.points = o3d.utility.Vector3dVector(np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]))
        axis.lines = o3d.utility.Vector2iVector(np.array([
            [0, 1],
            [0, 2],
            [0, 3]]))
        axis.colors = o3d.utility.Vector3dVector(np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]))
        vis.add_geometry(axis)


    def semantic_lidar_callback(self, point_cloud, point_list):
        """Prepares a point cloud with semantic segmentation
        colors ready to be consumed by Open3D"""
        data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
            ("x", np.float32), ("y", np.float32), ("z", np.float32),
            ("CosAngle", np.float32), ("ObjIdx", np.uint32), ("ObjTag", np.uint32)]))

        # We"re negating the y to correclty visualize a world that matches
        # what we see in Unreal since Open3D uses a right-handed coordinate system
        points = np.array([data["x"], -data["y"], data["z"]]).T

        # # An example of adding some noise to our data if needed:
        # points += np.random.uniform(-0.05, 0.05, size=points.shape)

        # Colorize the pointcloud based on the CityScapes color palette
        labels = np.array(data["ObjTag"])
        int_color = SensorlibIntegrationTestRunner.LABEL_COLORS[labels]

        # # In case you want to make the color intensity depending
        # # of the incident ray angle, you can use:
        # int_color *= np.array(data["CosAngle"])[:, None]

        point_list.points = o3d.utility.Vector3dVector(points)
        point_list.colors = o3d.utility.Vector3dVector(int_color)