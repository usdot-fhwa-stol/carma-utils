from collections import deque


class SensorDataCollector:

    def __init__(self, carla_world, carla_sensor):
        self.__carla_world = carla_world
        self.__carla_sensor = carla_sensor
        self.__prev_angle = 0.0
        # Store current and prior data collections. The current is actively being added to, previous is finalized.
        self.__data = deque([[], []], maxlen=2)

    def get_carla_lidar_hitpoints(self):
        # Prior collection is always complete
        return self.__data[1]

    def __collect_sensor_data(self, sensor_rotation_angle, raw_sensor_data):
        if not self.__is_same_data_collection(sensor_rotation_angle):
            # Finalize current collection and append a new collection
            self.__data.appendleft([])

        # Add data to the current collection
        data = self.__data[0]
        data += raw_sensor_data

    # Determines if this data collection belongs to the same data collection run as the previous time step
    def __is_same_data_collection(self, sensor_rotation_angle):
        is_increasing = sensor_rotation_angle >= self.__prev_angle
        self.__prev_angle = sensor_rotation_angle
        return is_increasing
