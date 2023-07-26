import numpy as np

class CarlaUtils:

    @staticmethod
    def vector3d_to_numpy(vec):
        np.array( [vec.x, vec.y, vec.z] )

    @staticmethod
    def vector2d_to_numpy(vec):
        np.array( [vec.x, vec.y] )

