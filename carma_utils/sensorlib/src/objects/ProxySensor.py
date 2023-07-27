import numpy as np

from src.util.CarlaUtils import CarlaUtils


class ProxySensor:
    def __init__(self, carla_sensor):
        self.__carla_sensor = carla_sensor

    def get_position(self):
        return CarlaUtils.vector3d_to_numpy(self.__carla_sensor.get_location())

    def get_points_per_second(self):
        return self.__carla_sensor.points_per_second

    def get_rotation_frequency(self):
        return self.__carla_sensor.rotation_frequency

    def get_fov_angular_width(self):
        return np.deg2rad(self.__carla_sensor.upper_fov - self.__carla_sensor.lower_fov)
