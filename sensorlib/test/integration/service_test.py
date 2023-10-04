import unittest
from unittest.mock import MagicMock

from main.py import main

class TestService(unittest.TestCase):
    def test_main(self):

        SimulatedSensorConfigurator.register_simulated_semantic_lidar_sensor = MagicMock(return_value=0)

        # TODO


        self.assertEqual(main(), "Hello World!")



