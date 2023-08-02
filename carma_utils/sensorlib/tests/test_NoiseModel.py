import unittest
from unittest.mock import MagicMock
import numpy as np

from src.noise_models.GaussianNoiseModel import GaussianNoiseModel
from tests.SimulatedSensorTestUtils import SimulatedSensorTestUtils


class TestNoiseModelTest(unittest.TestCase):
    def setUp(self):
        self.config = SimulatedSensorTestUtils.generate_noise_model_config()
        self.noise_model = GaussianNoiseModel(self.config)
        self.detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_object()

    def test_apply_position_noise(self):
        object_list = [MagicMock(position=np.array([1.0, 2.0, 3.0])), MagicMock(position=np.array([4.0, 5.0, 6.0]))]
        np.random.normal = MagicMock(return_value=np.array([0.1, 0.2, 0.3]))

        self.noise_model.apply_position_noise(object_list)

        self.assertEqual(object_list[0].position.tolist(), [1.1, 2.2, 3.3])
        self.assertEqual(object_list[1].position.tolist(), [4.1, 5.2, 6.3])
        np.random.normal.assert_called_with(0.0, [0.8, 0.8, 0.8], size=3)

    def test_apply_orientation_noise(self):
        object_list = [MagicMock(rotation=np.array([[1.0, 2.0, 3.0, 4.0], [1.0, 2.0, 3.0, 4.0], [1.0, 2.0, 3.0, 4.0], [1.0, 2.0, 3.0, 4.0]])),
                       MagicMock(rotation=np.array([[5.0, 6.0, 7.0, 8.0], [5.0, 6.0, 7.0, 8.0], [5.0, 6.0, 7.0, 8.0], [5.0, 6.0, 7.0, 8.0]]))]

        np.random.normal = MagicMock(return_value=np.array([[0.1, 0.1, 0.1, 0.1], [0.1, 0.1, 0.1, 0.1], [0.1, 0.1, 0.1, 0.1], [0.1, 0.1, 0.1, 0.1]]))

        self.noise_model.apply_orientation_noise(object_list)

        self.assertEqual(object_list[0].rotation.tolist(),
                         [[1.1, 2.1, 3.1, 4.1], [1.1, 2.1, 3.1, 4.1], [1.1, 2.1, 3.1, 4.1], [1.1, 2.1, 3.1, 4.1]])
        self.assertEqual(object_list[1].rotation.tolist(),
                         [[5.1, 6.1, 7.1, 8.1], [5.1, 6.1, 7.1, 8.1], [5.1, 6.1, 7.1, 8.1], [5.1, 6.1, 7.1, 8.1]])

        np.random.normal.assert_called_with(0.0, 0.1, size=(4, 4))

    def test_apply_type_noise(self):
        object_list = [MagicMock(object_type="Pedestrian"), MagicMock(object_type="Vehicle")]

        np.random.sample = MagicMock(return_value="TestValue")

        self.noise_model.apply_type_noise(object_list)

        self.assertEqual(object_list[0].object_type, "TestValue")
        self.assertEqual(object_list[1].object_type, "TestValue")

        np.random.sample.assert_called_with(self.config["noise_model_config"]["type_noise"]["allowed_semantic_tags"], 1)

    def test_apply_list_inclusion_noise(self):
        object_list = [MagicMock(), MagicMock(), MagicMock()]

        np.random.randint = MagicMock(return_value=2)

        result = self.noise_model.apply_list_inclusion_noise(object_list)

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0], object_list[0])
        self.assertEqual(result[1], object_list[1])


if __name__ == '__main__':
    unittest.main()
