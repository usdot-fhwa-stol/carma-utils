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

import sched
import time
from xmlrpc.server import SimpleXMLRPCServer
import threading

from src.SimulatedSensorConfigurator import SimulatedSensorConfigurator
from src.util.SimulatedSensorUtils import SimulatedSensorUtils

import carla


def scheduled_compute(scheduler, simulated_lidar_sensor, detection_cycle_delay_seconds):
    scheduler.enter(detection_cycle_delay_seconds, 1, scheduled_compute)
    simulated_lidar_sensor.compute_detected_objects()


def main(infrastructure_id, sensor_config, noise_model_config, detection_cycle_delay_seconds,
         carla_host, carla_port,
         start_rpc_server, xmlrpc_server_host, xmlrpc_server_port,
         debug_mode=False):
    """
    Instantiate a SimulatedLidarSensor and start an XML-RPC server to provide detected objects.

    The sensor compute loop, in which detected objects are identified, is run continuously in a separate thread with
     a detection_cycle_delay_seconds loop delay.

    Two starting modes are supported:

        1. With start_rpc_server=True an RPC server exposes the `get_detected_objects_json()` function to retrieve
             the list of currently detected objects as a JSON string, and blocks further execution.
        2. Alternatively with start_rpc_server=False, the `get_detected_objects_json()` function may be called
            programmatically on the returned sensor object.

    :param infrastructure_id: Infrastructure ID to assign to the sensor. Negative value forces auto-assignment.
    :param sensor_config: Sensor configuration as either a file name, JSON dictionary, or blank.
                            - If the value is a valid .yaml file name, the file is loaded.
                            - If the value is a JSON dictionary, the dictionary is used directly.
                            - If passed as an empty string, default values are loaded from a configuration file.
    :param noise_model_config: Noise model configuration as either a file name, JSON dictionary, or blank.
                            - If the value is a valid .yaml file name, the file is loaded.
                            - If the value is a JSON dictionary, the dictionary is used directly.
                            - If passed as an empty string, default values are loaded from a configuration file.
    :param detection_cycle_delay_seconds: Delay in sensor computation loop (seconds).
    :param carla_host: CARLA host.
    :param carla_port: CARLA host port.
    :param start_rpc_server: Starts an XML-RPC server if True.
    :param xmlrpc_server_host: Host name for XML-RPC server.
    :param xmlrpc_server_port: Port for XML-RPC server.

    :return: No return if RPC server is running as this blocks execution; The SimulatedLidarSensor object if RPC server
                is not running.
    """
    # Get inputs
    sensor_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0),
                                       carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))

    # Retrieve the sensor configuration
    if os.path.exists(sensor_config):
        config = SimulatedSensorUtils.load_config_from_file(sensor_config)
    elif sensor_config is None or sensor_config == "":
        config = SimulatedSensorUtils.load_config_from_file("../config/simulated_sensor_config.yaml")
    else:
        config = json.loads(sensor_config)
    simulated_sensor_config = config["simulated_sensor"]
    carla_sensor_config = config["lidar_sensor"]

    # Retrieve noise model configuration
    if os.path.exists(noise_model_config):
        noise_model_config = SimulatedSensorUtils.load_config_from_file(noise_model_config)
    if noise_model_config is None or noise_model_config == "":
        noise_model_config = SimulatedSensorUtils.load_config_from_file("../config/noise_model_config.yaml")
    else:
        noise_model_config = json.loads(noise_model_config)

    # Build sensor
    simulated_lidar_sensor = SimulatedSensorConfigurator.register_simulated_semantic_lidar_sensor(
        simulated_sensor_config,
        carla_sensor_config,
        noise_model_config,
        carla_host, carla_port,
        sensor_transform,
        infrastructure_id,
        None,
        debug_mode)

    # Compute detected objects continuously using a separate thread
    scheduler = sched.scheduler(time.time, time.sleep)
    scheduler.enter(detection_cycle_delay_seconds, 1, scheduled_compute, (scheduler, simulated_lidar_sensor,
                                                                          detection_cycle_delay_seconds))
    scheduler_thread = threading.Thread(target=scheduler.run)
    print("Starting sensorlib compute.")
    scheduler_thread.start()

    # Create an XML-RPC server
    if start_rpc_server:
        print("Starting sensorlib XML-RPC server.")
        server = SimpleXMLRPCServer((xmlrpc_server_host, xmlrpc_server_port))
        server.register_function(simulated_lidar_sensor.get_detected_objects_json, "get_detected_objects_json")
        server.serve_forever()
    else:
        return simulated_lidar_sensor
# TypeError: scheduled_compute() missing 3 required positional arguments: 'scheduler', 'simulated_lidar_sensor', and 'detection_cycle_delay_seconds'


if __name__ == "__main__":
    # Parse arguments
    arg_parser = argparse.ArgumentParser(
        description=__doc__)

    arg_parser.add_argument(
        "--id",
        default=-1,
        type=int,
        help="Infrastructure ID to assign to the sensor. Negative value forces auto-assignment. (default: -1)")

    arg_parser.add_argument(
        "--sensor-config",
        default="",
        type=str,
        help="Sensor configuration as JSON dictionary. (default: \"\")")

    arg_parser.add_argument(
        "--noise-model-config",
        default="",
        type=str,
        help="Noise model configuration as JSON dictionary. (default: \"\")")

    arg_parser.add_argument(
        "--detection-cycle-delay-seconds",
        default=0.5,
        type=float,
        help="Delay in continuous sensor processing loop in seconds. (default: 0.5)")

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
        "--start-rpc-server",
        default=True,
        type=bool,
        help="Start the XML-RPC server. (default: True)")

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

    arg_parser.add_argument(
        "--debug",
        default=False,
        type=bool,
        help="Enable debugging mode, which bypassing processing to produce only 10 detected objects using the truth state. (default: False)")

    args = arg_parser.parse_args()

    main(args.id, args.sensor_config, args.noise_model_config, args.detection_cycle_delay_seconds,
         args.carla_host, args.carla_port,
         args.start_rpc_server, args.xmlrpc_server_host, args.xmlrpc_server_port,
         args.debug)
