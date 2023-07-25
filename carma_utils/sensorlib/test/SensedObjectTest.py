import carla
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
    carla_actor.attributes = MagicMock(return_value=dict())
    carla_actor.id = MagicMock(return_value="123")
    carla_actor.is_alive = MagicMock(return_value=True)
    carla_actor.parent = MagicMock(return_value=None)
    carla_actor.type_id = MagicMock(return_value="vehicle.ford.mustang")
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


# def test_constructor(carla_actor):  TODO
