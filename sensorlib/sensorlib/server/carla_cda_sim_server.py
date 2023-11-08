# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

from datetime import datetime

from util.simulated_sensor_utils import SimulatedSensorUtils


class CarlaCDASimServer:
    """
    Provides core API functionality, wrapped in a servable interface.
    """

    def __init__(self, sensor_api, sensor_config, noise_model_config, detection_cycle_delay_seconds):
        self.__api = sensor_api
        self.__sensor_config = sensor_config
        self.__noise_model_config = noise_model_config
        self.__detection_cycle_delay_seconds = detection_cycle_delay_seconds

    # ------------------------------------------------------------------------------
    # Configurations
    # ------------------------------------------------------------------------------

    def set_sensor_configuration(self, sensor_config):
        self.__sensor_config = sensor_config

    def set_noise_model_configuration(self, noise_model_config):
        self.__noise_model_config = noise_model_config

    def set_detection_cycle_delay_seconds(self, detection_cycle_delay_seconds):
        self.__detection_cycle_delay_seconds = detection_cycle_delay_seconds

    # ------------------------------------------------------------------------------
    # Sensor Creation
    # ------------------------------------------------------------------------------

    def create_simulated_semantic_lidar_sensor(self, infrastructure_id, sensor_id, sensor_position, sensor_rotation, parent_id=None):
        simulated_sensor = self.__api.create_simulated_semantic_lidar_sensor(self.__sensor_config["simulated_sensor"],
                                                                             self.__sensor_config["lidar_sensor"],
                                                                             self.__noise_model_config,
                                                                             self.__detection_cycle_delay_seconds,
                                                                             infrastructure_id, sensor_id,
                                                                             sensor_position, sensor_rotation,
                                                                             parent_id)
        return str(simulated_sensor.get_id())

    # ------------------------------------------------------------------------------
    # Sensor and Data Access
    # ------------------------------------------------------------------------------

    def get_simulated_sensor(self, infrastructure_id, sensor_id):
        sensor = self.__api.get_simulated_sensor(infrastructure_id, sensor_id)
        return str(sensor.get_id())

    def get_detected_objects(self, infrastructure_id, sensor_id):
        detected_objects = self.__api.get_detected_objects(infrastructure_id, sensor_id)
        return_json = str(SimulatedSensorUtils.serialize_to_json(detected_objects))
        return return_json

    def get_echo_response(self):
        return f"Echo response from sensorlib XML-RPC server at {datetime.now().isoformat()}"
