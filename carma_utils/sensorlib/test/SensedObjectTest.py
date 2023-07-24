import carla
import pytest
from unittest.mock import MagicMock

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
    carla_actor.position = MagicMock(return_value=[])
    return carla_actor

def test_determine_object_type(config, carla_actor):
    # Nominal case
    carla_actor.semantic_types = MagicMock(return_value=[ "Vehicles" ])
    assert "Vehicles" == SensedObject.__determine_object_type(config, carla_actor)

    # Nominal case
    carla_actor.semantic_types = MagicMock(return_value=[ "Pedestrian" ])
    assert "Pedestrian" == SensedObject.__determine_object_type(config, carla_actor)

    # Multiple types
    carla_actor.semantic_types = MagicMock(return_value=[ "Invalid Type", "Vehicles" ])
    assert "Vehicles" == SensedObject.__determine_object_type(config, carla_actor)

    # No allowed type
    carla_actor.semantic_types = MagicMock(return_value=[ "Invalid Type" ])
    assert "Unknown" == SensedObject.__determine_object_type(config, carla_actor)


# def test_constructor(carla_actor):  TODO
