import unittest


class TestCarlaActorUtils(unittest.TestCase):
    # TODO not verified

    def setUp(self):
        self.carla_actor = MagicMock()

    def test_determine_object_type(config, carla_actor):
        # Nominal case
        carla_actor.semantic_types = ["Vehicles"]
        assert "Vehicles" == SensedObject.determine_object_type(config, carla_actor)

        # Nominal case
        carla_actor.semantic_types = ["Pedestrian"]
        assert "Pedestrian" == SensedObject.determine_object_type(config, carla_actor)

        # Multiple types
        carla_actor.semantic_types = ["Invalid Type", "Vehicles"]
        assert "Vehicles" == SensedObject.determine_object_type(config, carla_actor)

        # No allowed type
        carla_actor.semantic_types = ["Invalid Type"]
        assert "Unknown" == SensedObject.determine_object_type(config, carla_actor)

    def test_get_actor_angular_velocity(self):
        expected_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.carla_actor.get_angular_velocity.return_value = MagicMock(x=0.0, y=0.0, z=0.0)
        result = CarlaActorUtils.get_actor_angular_velocity(self.carla_actor)
        self.assertTrue(np.array_equal(result, expected_angular_velocity))

    def test_get_actor_rotation_matrix(self):
        expected_rotation_matrix = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        self.carla_actor.get_transform.return_value = MagicMock(
            get_matrix=MagicMock(return_value=expected_rotation_matrix))
        result = CarlaActorUtils.get_actor_rotation_matrix(self.carla_actor)
        self.assertTrue(np.array_equal(result, expected_rotation_matrix))

    def test_get_actor_rotation_matrix(self):
        # Create a mock carla_actor object
        class MockCarlaActor:
            def get_transform(self):
                class MockTransform:
                    def get_matrix(self):
                        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

                return MockTransform()

        # Call the get_actor_rotation_matrix function
        carla_actor = MockCarlaActor()
        result = CarlaActorUtils.get_actor_rotation_matrix(carla_actor)

        # Check if the result is correct
        expected_result = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.assertEqual(result, expected_result)

    def test_get_actor_bounding_size(self):
        expected_bounding_size = np.array([0.0, 0.0, 0.0])
        self.carla_actor.get_bounding_box.return_value = MagicMock(extent=MagicMock(x=0.0, y=0.0, z=0.0))
        result = CarlaActorUtils.get_actor_bounding_size(self.carla_actor)
        self.assertTrue(np.array_equal(result, expected_bounding_size))


if __name__ == '__main__':
    unittest.main()
