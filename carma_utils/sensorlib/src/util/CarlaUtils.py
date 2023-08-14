import numpy as np
from scipy.spatial.transform import Rotation


class CarlaUtils:
    """
    Generic CARLA utility functions.
    """

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
        carla_rotation = carla_actor.get_transform().rotation
        rotation_angles_deg = np.array([carla_rotation.roll, carla_rotation.pitch, carla_rotation.yaw])
        rotation_angles = np.deg2rad(rotation_angles_deg)
        rotation_matrix = Rotation.from_euler('xyz', rotation_angles)
        return rotation_matrix.as_matrix()

    @staticmethod
    def get_actor_bounding_box_points(carla_actor):
        bounding_box_locations = carla_actor.get_world_vertices(carla_actor.get_transform())
        return [CarlaUtils.vector3d_to_numpy(location) for location in bounding_box_locations]

    @staticmethod
    def determine_object_type(carla_actor, allowed_semantic_tags):
        """
        Check for identification as one of the accepted types, and mark unidentified otherwise.
        Args:

        """

        # Set intersection, except order matters
        for tag in allowed_semantic_tags:
            if tag in carla_actor.semantic_tags:
                return tag
        return "Unknown"
