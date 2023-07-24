import pytest
from unittest.mock import MagicMock

def inc(x):
    return x + 1

def test_answer():
    assert inc(3) == 4

class CarlaSensor:
    def __init__(self):
        pass

@pytest.fixture
def carla_sensor_instance():
    carla_sensor = CarlaSensor
    carla_sensor.get_position = MagicMock(return_value=(1,2,3))
    return carla_sensor

def test_carla_sensor(carla_sensor_instance):
    assert (1,2,3) == carla_sensor_instance.get_position()
