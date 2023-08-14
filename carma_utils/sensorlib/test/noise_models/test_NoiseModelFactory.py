import unittest

from src.noise_models.GaussianNoiseModel import GaussianNoiseModel
from src.noise_models.NoiseModelFactory import NoiseModelFactory
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestNoiseModelFactory(unittest.TestCase):
    def setUp(self):
        self.config = SimulatedSensorTestUtils.generate_noise_model_config()

    def test_get_noise_model(self):
        noise_model = NoiseModelFactory.get_noise_model("GaussianNoiseModel", self.config)
        self.assertIsInstance(noise_model, GaussianNoiseModel)
