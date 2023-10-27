# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import unittest

from noise_models.GaussianNoiseModel import GaussianNoiseModel
from noise_models.NoiseModelFactory import NoiseModelFactory
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestNoiseModelFactory(unittest.TestCase):
    def setUp(self):
        self.config = SimulatedSensorTestUtils.generate_noise_model_config()

    def test_get_noise_model(self):
        noise_model = NoiseModelFactory.get_noise_model("GaussianNoiseModel", self.config)
        self.assertIsInstance(noise_model, GaussianNoiseModel)
