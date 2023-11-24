# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import unittest
from unittest.mock import MagicMock

import numpy as np

from noise_models.GaussianNoiseModel import GaussianNoiseModel
from test.util.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestGaussianNoiseModel(unittest.TestCase):
    def setUp(self):
        self.config = SimulatedSensorTestUtils.generate_noise_model_config()

    def test_apply_position_noise(self):
        object_list = [MagicMock(position=np.array([1.0, 2.0, 3.0])), MagicMock(position=np.array([4.0, 5.0, 6.0]))]
        np.random.normal = MagicMock(return_value=np.array([0.1, 0.2, 0.3]))

        noise_model = GaussianNoiseModel(self.config)

        noise_model.apply_position_noise(object_list)

        self.assertEqual(object_list[0].position.tolist(), [1.1, 2.2, 3.3])
        self.assertEqual(object_list[1].position.tolist(), [4.1, 5.2, 6.3])
        np.random.normal.assert_called_with(0.0, [0.8, 0.8, 0.8], size=3)

    def test_apply_orientation_noise(self):
        object_list = [
            MagicMock(rotation=np.array([[1.0, 2.0, 3.0], [1.0, 2.0, 3.0], [1.0, 2.0, 3.0], [1.0, 2.0, 3.0]])),
            MagicMock(rotation=np.array([[5.0, 6.0, 7.0], [5.0, 6.0, 7.0], [5.0, 6.0, 7.0], [5.0, 6.0, 7.0]]))]

        np.random.normal = MagicMock(
            return_value=np.array([[0.1, 0.1, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]]))

        noise_model = GaussianNoiseModel(self.config)

        noise_model.apply_orientation_noise(object_list)

        self.assertEqual(object_list[0].rotation.tolist(),
                         [[1.1, 2.1, 3.1], [1.1, 2.1, 3.1], [1.1, 2.1, 3.1], [1.1, 2.1, 3.1]])
        self.assertEqual(object_list[1].rotation.tolist(),
                         [[5.1, 6.1, 7.1], [5.1, 6.1, 7.1], [5.1, 6.1, 7.1], [5.1, 6.1, 7.1]])

        np.random.normal.assert_called_with(0.0, [0.1, 0.1, 0.1], size=3)

    def test_apply_type_noise(self):
        object_list = SimulatedSensorTestUtils.generate_test_data_detected_objects()

        np.random.default_rng = MagicMock(return_value=MagicMock(choice=MagicMock(return_value=4)))

        noise_model = GaussianNoiseModel(self.config)

        object_list = noise_model.apply_type_noise(object_list)

        self.assertEqual(object_list[0].type, "PEDESTRIAN")
        self.assertEqual(object_list[1].type, "PEDESTRIAN")
        self.assertEqual(object_list[2].type, "PEDESTRIAN")
        self.assertEqual(object_list[3].type, "PEDESTRIAN")
        self.assertEqual(object_list[4].type, "PEDESTRIAN")
        self.assertEqual(object_list[5].type, "PEDESTRIAN")

    def test_apply_list_inclusion_noise(self):
        object_list = [MagicMock(), MagicMock(), MagicMock()]

        # Rebuild noise model instance with mocked random generator
        np.random.randint = MagicMock(return_value=2)
        noise_model = GaussianNoiseModel(self.config)

        result = noise_model.apply_list_inclusion_noise(object_list)

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0], object_list[0])
        self.assertEqual(result[1], object_list[1])
