# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import argparse
import threading

from xmlrpc.server import SimpleXMLRPCServer

from CarlaCDASimAPI import CarlaCDASimAPI
from util.SimulatedSensorUtils import SimulatedSensorUtils

class CarlaCDASimAdapter:

    def __init__(self, sensor_api):
        """
        CarlaCDASimAdapter constructor.
        :param sensor_api: The API object exposing CARLA connection.
        """
        self.__api = sensor_api

    def start_xml_rpc_server(self, xmlrpc_server_host, xmlrpc_server_port, blocking=True):
        """
        Starts the XML-RPC server for the sensor data service.

        :param xmlrpc_server_host: The server host.
        :param xmlrpc_server_port: The server port.
        :param blocking: Toggles server starting on main thread. If False the server will be started on a new thread, and control will be returned to the caller.
        :return: Thread object the server was started on, or blocked if the server was started on the main thread.
        """

        # Create an XML-RPC server
        print("Starting sensorlib XML-RPC server.")
        server = SimpleXMLRPCServer((xmlrpc_server_host, xmlrpc_server_port))
        server.register_introspection_functions()
        server.register_function(self.__create_simulated_semantic_lidar_sensor,
                                 "create_simulated_semantic_lidar_sensor")
        server.register_function(self.__get_simulated_sensor, "get_simulated_sensor")
        server.register_function(self.__get_detected_objects, "get_detected_objects")

        # Start, with blocking option
        if blocking:
            return server.serve_forever()
        else:
            rpc_server_thread = threading.Thread(target=server.serve_forever)
            rpc_server_thread.start()
            return rpc_server_thread

    def __create_simulated_semantic_lidar_sensor(self, sensor_config_file, noise_model_config_file,
                                                 detection_cycle_delay_seconds,
                                                 infrastructure_id, sensor_id,
                                                 sensor_position, sensor_rotation, parent_actor_id):
        sensor_config = SimulatedSensorUtils.load_config_from_file(sensor_config_file)
        noise_model_config = SimulatedSensorUtils.load_config_from_file(noise_model_config_file)

        simulated_sensor = self.__api.create_simulated_semantic_lidar_sensor(sensor_config["simulated_sensor"],
                                                                             sensor_config["lidar_sensor"],
                                                                             noise_model_config,
                                                                             detection_cycle_delay_seconds,
                                                                             infrastructure_id, sensor_id,
                                                                             sensor_position, sensor_rotation,
                                                                             parent_actor_id)
        return str(simulated_sensor.get_id())

    def __get_simulated_sensor(self, infrastructure_id, sensor_id):
        sensor = self.__api.get_simulated_sensor(infrastructure_id, sensor_id)
        return str(sensor.get_id())

    def __get_detected_objects(self, infrastructure_id, sensor_id):
        detected_objects = self.__api.get_detected_objects(infrastructure_id, sensor_id)
        return_json = str(SimulatedSensorUtils.serialize_to_json(detected_objects))
        return return_json


if __name__ == "__main__":
    # Parse arguments
    arg_parser = argparse.ArgumentParser(
        description=__doc__)

    arg_parser.add_argument(
        "--carla-host",
        default="127.0.0.1",
        type=str,
        help="CARLA host. (default: \"localhost\")")

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
    sensor_api = CarlaCDASimAPI.build_from_host_spec(args.carla_host, args.carla_port)
    sensor_data_service = CarlaCDASimAdapter(sensor_api)
    sensor_data_service.start_xml_rpc_server(args.xmlrpc_server_host, args.xmlrpc_server_port, True)
