from collections import deque

import numpy as np

from src.util.CarlaUtils import CarlaUtils


class SensorDataCollector:

    def __init__(self, carla_world, carla_sensor):
        self.__carla_world = carla_world
        self.__carla_sensor = carla_sensor
        self.__prev_angle = 0.0

        # Store current and prior data collections. The current is actively being added to, previous is finalized.
        self.__data = deque([[], []], maxlen=2)

        # Register callback to collect data
        self.__carla_sensor.listen(self.__collect_sensor_data)

    def get_carla_lidar_hitpoints(self):
        # Prior collection is always complete
        return self.__data[1]

    def __collect_sensor_data(self, raw_sensor_data):
        # Check if this data collection belongs to the same data collection run as the previous time step
        sensor_rotation_angle = raw_sensor_data.horizontal_angle
        print(sensor_rotation_angle)
        if not self.__is_same_data_collection(sensor_rotation_angle):
            # Finalize current collection and append a new collection
            self.__data.appendleft({})

        # Add data to the current collection
        self.__collect_raw_point_data(self.__data[0], raw_sensor_data)

    def __collect_raw_point_data(self, grouped_data, raw_sensor_data):
        # Extract geometric hitpoints and group them by actor ID
        # The resulting dictionary maps actor ID to a list of hitpoints
        new_data = [( r.object_idx, [np.array(r.hit_point)] )
                    for r in raw_sensor_data]

        combined_data_list = list(grouped_data.items()) + new_data

        # Merge new data into the old dataset
        grouped_data += accumulate(
            combined_data_list,
            lambda old_hitpoints, new_hitpoints: old_hitpoints + new_hitpoints,
        )

    # Determines if this data collection belongs to the same data collection run as the previous time step
    def __is_same_data_collection(self, sensor_rotation_angle):
        is_increasing = sensor_rotation_angle > self.__prev_angle
        self.__prev_angle = sensor_rotation_angle
        return is_increasing
