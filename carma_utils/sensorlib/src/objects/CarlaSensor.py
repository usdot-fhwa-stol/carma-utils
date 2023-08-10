from dataclasses import dataclass

import carla
import numpy as np

from src.util.CarlaUtils import CarlaUtils

@dataclass(frozen=True)
class CarlaSensor:
    carla_sensor: carla.Sensor
    position: np.ndarray
    points_per_second: float
    rotation_frequency: float
    fov_angular_width: float


class CarlaSensorBuilder:
    @staticmethod
    def build_sensor(carla_sensor):
        return CarlaSensor(carla_sensor,
                           CarlaUtils.vector3d_to_numpy(carla_sensor.get_location()),
                           carla_sensor.points_per_second,
                           carla_sensor.rotation_frequency,
                           np.deg2rad(carla_sensor.upper_fov - carla_sensor.lower_fov))
