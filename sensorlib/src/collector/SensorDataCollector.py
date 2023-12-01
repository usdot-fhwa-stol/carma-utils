# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

from collections import deque

from util.CarlaUtils import CarlaUtils


class SensorDataCollector:
    """
    Collects data from the sensor via CARLA's sensor callback feature.

    Hitpiont data specifically from the CARLA Semantic LIDAR Sensor is associated and grouped by actor ID,
    which is beneficial to preserve.

    Hitpoints are stored inside a two-element circular queue, with __data[0] acting as the active object collection
    and _data[1] the collection from the prior scan. As the LIDAR sensor scans, it may periodically activate the
    callback. When this happens, data is appended to that inside the active collection __data[0]. When a new scan is
    detected, the collections are rotated making __data[0] the prior and an empty __data[0] the current.

    New data scans are detected by a reset in the sensor read angle as reported by the simulator.
    """

    def __init__(self, carla_world, carla_sensor):
        self.debug = True
        self.__carla_world = carla_world
        self.__carla_sensor = carla_sensor
        self.__prev_angle = 0.0

        # Time of latest data capture (in seconds)
        self.__timestamp = 0.0

        # Store current and prior data collections. The current is actively being added to, previous is finalized.
        self.__data = deque([{}, {}], maxlen=2)

        # Register callback to collect data
        self.__carla_sensor.listen(self.__collect_sensor_data)

    def get_carla_lidar_hitpoints(self):
        """
        Prior collection is always complete.

        :return: Timestamp (in seconds), and collected hitpoints from most recent frame (dictionary mapping object ID
            to list of np.array points).
        """
        return self.__timestamp, self.__data[1]

    def __collect_sensor_data(self, semantic_sensor_data):
        """
        Primary function, registered as the callback for the CARLA sensor. This function is called whenever CARLA has
        updated data. Data is collected and added to the active collection. Collections are rotated when current
        collection is complete, as determined by sensor rotation angle.

        :param semantic_sensor_data: Measurement from CARLA, carla.SemanticLidarMeasurement.
        :return: None
        """

        # Update the timestamp (in float seconds)
        self.__timestamp = semantic_sensor_data.timestamp

        # Check if this data collection belongs to the same data collection run as the previous time step
        sensor_rotation_angle = semantic_sensor_data.horizontal_angle
        if not self.__is_same_data_collection(sensor_rotation_angle):
            # Finalize current collection and append a new collection
            self.__data.appendleft({})

        # Add data to the current collection
        if (len(semantic_sensor_data.raw_data) == 0):
            return None

        self.__collect_raw_point_data(self.__data[0], semantic_sensor_data)

    def __collect_raw_point_data(self, grouped_data, raw_sensor_data):
        """
        Collect the raw hit point data from the measurement.

        :param grouped_data: Current active collection being appended to.
        :param raw_sensor_data: Measurement from CARLA, carla.SemanticLidarMeasurement.
        :return: None
        """

        # Extract geometric hitpoints and group them by actor ID
        # The resulting dictionary maps actor ID to a list of hitpoints
        for detection in raw_sensor_data:
            # Skip if object_idx is 0 as it is static objects like buildings (it represents actor_id in new versions 0.9.11+)
            # and a large number of hitpoints which creates performance issues for this library's nearest neighbor algorithm
            # https://github.com/carla-simulator/carla/issues/3191
            if (detection.object_idx == 0):
                continue

            point = CarlaUtils.vector3d_to_numpy(detection.point)

            # CARLA 0.9.10 has a bug where the y-axis value is negated.
            # This was resolved in a later release, but CARMA currently
            # uses 0.9.10. Remove this fix when CARMA upgrades to a
            # newer CARLA version.
            point[1] *= -1

            if detection.object_idx not in grouped_data:
                grouped_data[detection.object_idx] = [point]
            else:
                grouped_data[detection.object_idx].append(point)

    def __is_same_data_collection(self, sensor_rotation_angle):
        """
        Determine if this data collection belongs to the same data collection run as the previous time step.

        :param sensor_rotation_angle: LIDAR sensor angular position at time of data collection.
        :return: True if new data belongs to current collection, False if it belongs to the next collection.
        """
        is_increasing = sensor_rotation_angle > self.__prev_angle
        self.__prev_angle = sensor_rotation_angle
        return is_increasing
