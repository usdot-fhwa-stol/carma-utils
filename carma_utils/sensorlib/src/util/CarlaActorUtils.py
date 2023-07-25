from src.util.CarlaUtils import CarlaUtils
import numpy as np

class CarlaActorUtils:

    DEG_TO_RAD = np.pi / 360.0

    @staticmethod
    def get_actor_angular_velocity(carla_actor):
        """
        Get carla.Actor angular velocity in radians per second.
        :param carla_actor:
        :return: numpy.array containing angular velocity vector in radians per second.
        """
        angular_velocity_degpersecond = CarlaUtils.vector3d_to_numpy(carla_actor.get_angular_velocity())
        return angular_velocity_degpersecond * CarlaActorUtils.DEG_TO_RAD
