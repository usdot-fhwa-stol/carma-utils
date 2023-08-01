import numpy as np


class CarlaUtils:

    @staticmethod
    def vector3d_to_numpy(vec):
        """
        Convert a carla.Vector3D to a numpy array.
        :param vec: carla.Vector3D to convert.
        :return: Vector in numpy array form.
        """
        return np.array([vec.x, vec.y, vec.z])

    @staticmethod
    def vector2d_to_numpy(vec):
        """
        Convert a carla.Vector2D to a numpy array.
        :param vec: carla.Vector2D to convert.
        :return: Vector in numpy array form.
        """
        return np.array([vec.x, vec.y])

    @staticmethod
    def get_actor_angular_velocity(carla_actor):
        """
        Get carla.Actor angular velocity in radians per second.
        :param carla_actor:
        :return: numpy.array containing angular velocity vector in radians per second.
        """
        angular_velocity_degpersecond = CarlaUtils.vector3d_to_numpy(carla_actor.get_angular_velocity())
        return np.deg2rad(angular_velocity_degpersecond)

    @staticmethod
    def get_actor_rotation_matrix(carla_actor):
        rotation_matrix = carla_actor.get_transform().get_matrix()
        rotation_matrix = np.array(rotation_matrix)
        rotation_matrix = np.deg2rad(rotation_matrix)
        return rotation_matrix

    @staticmethod
    def get_actor_bounding_size(carla_actor):
        extent_vector = carla_actor.get_actor_bounding_size().extent
        # Extent vector is half the length, width, and height of the bounding box
        return 2.0 * np.array([extent_vector.x, extent_vector.y, extent_vector.z])
