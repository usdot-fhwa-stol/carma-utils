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

from carla_cda_sim_api import CarlaCDASimAPI
from server.carla_cda_sim_server import CarlaCDASimServer
from util.simulated_sensor_utils import SimulatedSensorUtils


class CarlaCDASimAdapter:
    """
    Maps command line arguments into the core server functionality.
    """

    def __init__(self, sensor_api, sensor_config_file="config/simulated_sensor_config.yaml",
                 noise_model_config_file="config/noise_model_config.yaml", detection_cycle_delay_seconds=0.5):
        """
        CarlaCDASimAdapter constructor.
        :param sensor_api: The API object exposing CARLA connection.
        """

        # Get full file paths
        sensor_config_full_path = SimulatedSensorUtils.get_root_path(sensor_config_file)
        noise_model_config_full_path = SimulatedSensorUtils.get_root_path(noise_model_config_file)

        # Load default configurations
        sensor_config = SimulatedSensorUtils.load_config_from_file(sensor_config_full_path)
        noise_model_config = SimulatedSensorUtils.load_config_from_file(noise_model_config_full_path)

        # Build the server object
        self.__sim_server = CarlaCDASimServer(sensor_api, sensor_config, noise_model_config,
                                              detection_cycle_delay_seconds)

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

        # Register the configuration accessors
        server.register_function(self.__sim_server.set_sensor_configuration, "set_sensor_configuration")
        server.register_function(self.__sim_server.set_noise_model_configuration, "set_noise_model_configuration")
        server.register_function(self.__sim_server.set_detection_cycle_delay_seconds,
                                 "set_detection_cycle_delay_seconds")

        # Register sensor creation functions
        server.register_function(self.__sim_server.create_simulated_semantic_lidar_sensor,
                                 "create_simulated_semantic_lidar_sensor")

        # Register sensor and data access functions
        server.register_function(self.__sim_server.get_simulated_sensor, "get_simulated_sensor")
        server.register_function(self.__sim_server.get_detected_objects, "get_detected_objects")
        server.register_function(self.__sim_server.get_echo_response, "test.echo")

        # Start, with blocking option
        if blocking:
            return server.serve_forever()
        else:
            rpc_server_thread = threading.Thread(target=server.serve_forever)
            rpc_server_thread.start()
            return rpc_server_thread


if __name__ == "__main__":

    # Parse arguments
    arg_parser = argparse.ArgumentParser(description=__doc__)

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
        default="127.0.0.1",
        type=str,
        help="XML-RPC server host. (default: \"127.0.0.1\")")

    arg_parser.add_argument(
        "--xmlrpc-server-port",
        default=8000,
        type=int,
        help="XML-RPC server port. (default: 8000)")

    arg_parser.add_argument(
        "--sensor-config-file",
        default="config/simulated_sensor_config.yaml",
        type=str,
        help="Path to sensor configuration file. (default: config/simulated_sensor_config.yaml)")

    arg_parser.add_argument(
        "--noise-model-config-file",
        default="config/noise_model_config.yaml",
        type=str,
        help="Path to noise mode configuration file. (default: config/noise_model_config.yaml)")

    arg_parser.add_argument(
        "--detection-cycle-delay-seconds",
        default=0.5,
        type=float,
        help="Time interval between detection reporting. (default: 0.5)")

    args = arg_parser.parse_args()
    sensor_api = CarlaCDASimAPI.build_from_host_spec(args.carla_host, args.carla_port)
    sensor_data_service = CarlaCDASimAdapter(sensor_api, args.sensor_config_file, args.noise_model_config_file,
                                             args.detection_cycle_delay_seconds)
    sensor_data_service.start_xml_rpc_server(args.xmlrpc_server_host, args.xmlrpc_server_port, True)
