import unittest
import numpy as np
from unittest.mock import MagicMock

from src.util.CarlaUtils import CarlaUtils
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestCarlaUtils(unittest.TestCase):

    def setUp(self):
        self.carla_actor = MagicMock()

    def test_determine_object_type(config, carla_actor):
