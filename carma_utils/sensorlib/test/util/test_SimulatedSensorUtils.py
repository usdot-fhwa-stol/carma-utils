import os
import unittest

import yaml

from src.util.SimulatedSensorUtils import SimulatedSensorUtils
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestCarlaUtils(unittest.TestCase):

    def test_load_config_from_file(self):
        config_file_path = "test__simulated_sensor_config.yaml"
        config = SimulatedSensorTestUtils.generate_simulated_sensor_config()

        with open(config_file_path, "w") as file:
            yaml.dump(config, file)

        loaded_config = SimulatedSensorUtils.load_config_from_file(config_file_path)

        self.assertEqual(config, loaded_config)
        os.remove(config_file_path)
