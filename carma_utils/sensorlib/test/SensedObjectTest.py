import carla
import numpy as np
import pytest
from unittest.mock import MagicMock

from src.SensedObject import SensedObject


@pytest.fixture
def config():
    return {
        "prefilter": {
            "allowed_semantic_tags": [
                "Pedestrian",
                "Vehicles",
                "Other Allowed Type"
            ]
        }
    }


@pytest.fixture
def carla_actor():
    carla_actor = carla.Actor
    carla_actor.attributes = dict()
    carla_actor.id = 123
    carla_actor.is_alive = True
    carla_actor.parent = None
    carla_actor.semantic_types = ["Vehicles"]
    carla_actor.type_id = "vehicle.ford.mustang"

    carla_actor.get_acceleration = MagicMock(return_value=carla.Vector3D(0.0, 0.0, 0.0))
    carla_actor.get_angular_velocity = MagicMock(return_value=carla.Vector3D(0.0, 0.0, 0.005))
    carla_actor.get_location = MagicMock(return_value=carla.Location(10.0, 15.0, 7.0))
    carla_actor.get_transform = MagicMock(
        return_value=carla.Transform(carla.Location(10.0, 15.0, 7.0), carla.Rotation(3.0, 1.4, 4.0)))
    carla_actor.get_velocity = MagicMock(return_value=carla.Vector3D(100.0, 1.0, 0.0))
    carla_actor.get_world = MagicMock(return_value=carla.World)

    return carla_actor


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


def test_constructor(carla_actor):
    obj = SensedObject(config, carla_actor)
    assert 123 == obj.id
    assert "Vehicles" == obj.object_type

    assert np.array([10.0, 15.0, 7.0]) == obj.position

    expected_rotation = np.array([[0.05235987755983, 0.0, 0.0],
                                  [0.0, 0.024434609527921, 0.0],
                                  [0.0, 0.0, 0.698131700797732]])
    assert np.array_equal(expected_rotation, obj.rotation)
    assert np.array([100.0, 1.0, 0.0]) == obj.velocity
    assert np.array([0.0, 0.0, 0.005]) == obj.angular_velocity

    assert np.array([[4.0, 0.0, 0.0], [0.0, 4.0, 0.0], [0.0, 0.0, 4.0]]) == obj.position_covariance
    assert np.array([[4.0, 0.0, 0.0], [0.0, 4.0, 0.0], [0.0, 0.0, 4.0]]) == obj.velocity_covariance

    assert np.array([4.0, 3.0, 2.0]) == obj.size
    assert 1.0 == obj.confidence
