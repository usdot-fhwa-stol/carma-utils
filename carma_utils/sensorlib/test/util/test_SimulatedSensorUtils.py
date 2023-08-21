# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import os
import unittest

import yaml

from src.util.SimulatedSensorUtils import SimulatedSensorUtils
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestSimulatedSensorUtils(unittest.TestCase):

    def test_load_config_from_file(self):
        config_file_path = "test__simulated_sensor_config.yaml"
        config = SimulatedSensorTestUtils.generate_simulated_sensor_config()

        with open(config_file_path, "w") as file:
            yaml.dump(config, file)

        loaded_config = SimulatedSensorUtils.load_config_from_file(config_file_path)

        self.assertEqual(config, loaded_config)
        os.remove(config_file_path)
