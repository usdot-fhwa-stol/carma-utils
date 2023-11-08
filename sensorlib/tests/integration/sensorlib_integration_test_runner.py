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

import carla

from sensorlib.carla_cda_sim_api import CarlaCDASimAPI
from tests.integration.integration_test_utilities import IntegrationTestUtilities


class SensorlibIntegrationTestRunner(unittest.TestCase):

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
