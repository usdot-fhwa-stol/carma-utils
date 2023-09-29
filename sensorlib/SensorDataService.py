# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import argparse
import json
import os

from xmlrpc.server import SimpleXMLRPCServer

from SensorAPI import SensorAPI
from util.SimulatedSensorUtils import SimulatedSensorUtils

import carla


class SensorDataService:

    def __init__(self):
        self.__api = None

    def main(self, carla_host, carla_port, xmlrpc_server_host, xmlrpc_server_port):
        # Instantiate a SensorAPI
        self.__api = SensorAPI(carla_host, carla_port)

        # Create an XML-RPC server
        print("Starting sensorlib XML-RPC server.")
        server = SimpleXMLRPCServer((xmlrpc_server_host, xmlrpc_server_port))
        server.register_introspection_functions()
        server.register_function(self.__create_simulated_semantic_lidar_sensor,
                                 "create_simulated_semantic_lidar_sensor")
        server.register_function(self.__get_detected_objects, "get_detected_objects")
        server.serve_forever()

    def __create_simulated_semantic_lidar_sensor(self, sensor_config_file, noise_model_config_file,
                                                 detection_cycle_delay_seconds,
                                                 infrastructure_id, sensor_id,
                                                 sensor_position, sensor_rotation, parent_actor_id):
        print(f"__create_simulated_semantic_lidar_sensor {sensor_config_file} {noise_model_config_file} {detection_cycle_delay_seconds} {infrastructure_id} {sensor_id} {sensor_position} {sensor_rotation} {parent_actor_id}")
        return "__create_simulated_semantic_lidar_sensor"

        # sensor_config = SimulatedSensorUtils.load_config_from_file(sensor_config_file)
        # noise_model_config = SimulatedSensorUtils.load_config_from_file(noise_model_config_file)
        #
        # simulated_sensor = self.__api.create_simulated_semantic_lidar_sensor(sensor_config["simulated_sensor"],
        #                                                               sensor_config["lidar_sensor"], noise_model_config,
        #                                                               detection_cycle_delay_seconds,
        #                                                               infrastructure_id, sensor_id,
        #                                                               sensor_position, sensor_rotation, parent_actor_id)
        # return simulated_sensor.get_id()

    def __get_detected_objects(self, infrastructure_id, sensor_id):
        print(f"get_detected_objects {infrastructure_id} {sensor_id}")
        # detected_objects = self.__api.get_detected_objects(infrastructure_id, sensor_id)
        # return SimulatedSensorUtils.serialize_to_json(detected_objects)
        return "test"


if __name__ == "__main__":
    # Parse arguments
    arg_parser = argparse.ArgumentParser(
        description=__doc__)

    arg_parser.add_argument(
        "--infrastructure-id",
        type=int,
        help="Infrastructure ID to associate the sensor to.")

    arg_parser.add_argument(
        "--sensor-id",
        type=int,
        help="Sensor ID to assign.")

    arg_parser.add_argument(
        "--carla-host",
        default="127.0.0.1",
        type=str,
        help="CARLA host. (default: \"127.0.0.1\")")

    arg_parser.add_argument(
        "--carla-port",
        default="2000",
        type=str,
        help="CARLA host. (default: \"2000\")")

    arg_parser.add_argument(
        "--xmlrpc-server-host",
        default="localhost",
        type=str,
        help="XML-RPC server host. (default: \"localhost\")")

    arg_parser.add_argument(
        "--xmlrpc-server-port",
        default=8000,
        type=int,
        help="XML-RPC server port. (default: 8000)")

    args = arg_parser.parse_args()
    sensor_data_service = SensorDataService()
    sensor_data_service.main(args.carla_host, args.carla_port, args.xmlrpc_server_host, args.xmlrpc_server_port)
