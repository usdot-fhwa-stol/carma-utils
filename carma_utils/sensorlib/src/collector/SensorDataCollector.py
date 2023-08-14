from collections import deque

import numpy as np

from src.util.CarlaUtils import CarlaUtils


class SensorDataCollector:
    """
    Collects data from the sensor via CARLA's sensor callback feature.

    Hitpiont data specifically from the CARLA Semantic LIDAR Sensor is associated and grouped by actor ID, which is beneficial to preserve.

    Hitpoints are stored inside a two-element circular queue, with __data[0] acting as the active object collection and _data[1] the collection from the prior scan. As the LIDAR sensor scans, it may periodically activate the callback. When this happens, data is appended to that inside the active collection __data[0]. When a new scan is detected, the collections are rotated making __data[0] the prior and an empty __data[0] the current.

    New data scans are detected by a reset in the sensor read angle as reported by the simulator.
    """

    def __init__(self, carla_world, carla_sensor):
        self.debug = True
        self.__carla_world = carla_world
        self.__carla_sensor = carla_sensor
        self.__prev_angle = 0.0

        # Store current and prior data collections. The current is actively being added to, previous is finalized.
        self.__data = deque([{}, {}], maxlen=2)

        # Register callback to collect data
        self.__carla_sensor.listen(self.__collect_sensor_data)

    def get_carla_lidar_hitpoints(self):
        # Prior collection is always complete
        return self.__data[1]

    def __collect_sensor_data(self, raw_sensor_data):
        # Check if this data collection belongs to the same data collection run as the previous time step
        sensor_rotation_angle = raw_sensor_data.horizontal_angle
        if not self.__is_same_data_collection(sensor_rotation_angle):
            # Finalize current collection and append a new collection
            self.__data.appendleft({})

        # Add data to the current collection
        self.__collect_raw_point_data(self.__data[0], raw_sensor_data)

    def __collect_raw_point_data(self, grouped_data, raw_sensor_data):

        # Extract geometric hitpoints and group them by actor ID
        # The resulting dictionary maps actor ID to a list of hitpoints
        for r in raw_sensor_data.raw_data:
            point = CarlaUtils.vector3d_to_numpy(r.point)
            if r.object_idx not in grouped_data:
                grouped_data[r.object_idx] = [point]
            else:
                grouped_data[r.object_idx].append(point)

    # Determines if this data collection belongs to the same data collection run as the previous time step
    def __is_same_data_collection(self, sensor_rotation_angle):
        is_increasing = sensor_rotation_angle > self.__prev_angle
        self.__prev_angle = sensor_rotation_angle
        return is_increasing
