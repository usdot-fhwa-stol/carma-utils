# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

from abc import abstractmethod


class SimulatedSensor:
    """Sensor wrapper to contain logic analyzing internal CARLA sensor."""

    def __init__(self, infrastructure_id, sensor_id, sensor, parent_id):
        self.__infrastructure_id = infrastructure_id
        self.__sensor_id = sensor_id
        self.__sensor = sensor
        self.__parent_id = parent_id

    # ------------------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------------------

    def get_sensor(self):
        return self.__sensor

    def get_parent_id(self):
        return self.__parent_id

    def get_infrastructure_id(self):
        return self.__infrastructure_id

    def get_id(self):
        return self.__sensor_id

    # ------------------------------------------------------------------------------
    # Primary functions
    # ------------------------------------------------------------------------------

    @abstractmethod
    def compute_detected_objects(self):
        """
        Retrieve the sensor's latest perception of objects detected in the scene.
        :return: List of DetectedObject objects as Python objects.
        """
        pass

    @abstractmethod
    def get_detected_objects(self):
        """
        Retrieve the sensor's latest perception of objects detected in the scene.
        :return: List of DetectedObject objects.
        """
        pass
