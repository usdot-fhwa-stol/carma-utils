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
    carla_actor.semantic_types = [ "Vehicles" ]
    carla_actor.type_id = "vehicle.ford.mustang"
    return carla_actor

def test_determine_object_type(config, carla_actor):
    # Nominal case
    carla_actor.semantic_types = [ "Vehicles" ]
    assert "Vehicles" == SensedObject.determine_object_type(config, carla_actor)

    # Nominal case
    carla_actor.semantic_types = [ "Pedestrian" ]
    assert "Pedestrian" == SensedObject.determine_object_type(config, carla_actor)

    # Multiple types
    carla_actor.semantic_types = [ "Invalid Type", "Vehicles" ]
    assert "Vehicles" == SensedObject.determine_object_type(config, carla_actor)

    # No allowed type
    carla_actor.semantic_types = [ "Invalid Type" ]
    assert "Unknown" == SensedObject.determine_object_type(config, carla_actor)

def test_constructor(carla_actor):
    obj = SensedObject(config, carla_actor)
    assert 123 == obj.id
    assert "Vehicles" == obj.object_type
    assert np.array([ 10.0, 15.0, 7.0 ]) == obj.position
    assert np.array([ [ 1.0, 0.0, 0.0 ], [ 0.0, 1.0, 0.0 ], [ 0.0, 0.0, 1.0 ] ]) == obj.rotation
    assert np.array([ [ 4.0, 0.0, 0.0 ], [ 0.0, 4.0, 0.0 ], [ 0.0, 0.0, 4.0 ] ]) == obj.position_covariance
    assert np.array([ [ 4.0, 0.0, 0.0 ], [ 0.0, 4.0, 0.0 ], [ 0.0, 0.0, 4.0 ] ]) == obj.velocity_covariance
    assert np.array([ 100.0, 1.0, 0.0 ]) == obj.velocity
    assert np.array([ 0.0, 0.0, 0.005 ]) == obj.angular_velocity
    assert np.array([ 4.0, 3.0, 2.0 ]) == obj.size
    assert 0.9 == obj.confidence
