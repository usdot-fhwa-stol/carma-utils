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
from dataclasses import dataclass
from xmlrpc.server import SimpleXMLRPCServer
import threading

from SimulatedSensorConfigurator import SimulatedSensorConfigurator
from util.SimulatedSensorUtils import SimulatedSensorUtils

import carla


# TODO One RPC function to create a sensor at a given location/orientation.
# TODO One RPC function to retrieve all detected objects from the sensor.
# TODO Separate RPC server starting from all actions to build and run and expose the sensor capabilities.
# TODO Are configs consistent across sensors? If so only use one config.
# TODO Clarify transform absolute/relative if attached to a parent
# TODO Change noise model to toggle each noise model stage.
# TODO Change noise model to only change a subset of the output objects.
# TODO Turn noise model off with a different noise_model_name in the config.
# TODO remove machinet.conf
# TODO Remove debug parameters
# TODO Change infrastructure ID to use sub-id (sensor ID)
# TODO Sensor thread compute should be optional capability
# TODO Sensor management is not properly exposed in the interface (multiple sensors in the RPC)
# TODO Get_detected_objects has no ID parameter
# TODO Clarify interfaces int he readme

# TODO Get the xml rpc pr in place
# TODO Make a library PR (separate)

# TODO Expose configs to the RPC

# TODO Separate the service main.py from the library

# TODO Add a toggle to enable/disable the threading step
# TODO Share carla client connection between registered sensors
# TODO Move sensorlib sensor to be client to any RPC connection

# TODO Return ID or instance from the creation call in both interfaces

# TODO Make the service restartable by adding a search for existing sensors, and also add passive SimulatedSensor creation from existing CARLA sensor instance.

# TODO carla.Client(  ensure using non hardcoded parameters  )




def scheduled_compute(scheduler, simulated_lidar_sensor, detection_cycle_delay_seconds):
    scheduler.enter(detection_cycle_delay_seconds, 1, scheduled_compute)
    simulated_lidar_sensor.compute_detected_objects()


def main(infrastructure_id, sensor_config, noise_model_config, detection_cycle_delay_seconds,
         carla_host, carla_port,
         start_rpc_server, xmlrpc_server_host, xmlrpc_server_port,
         enable_processing=True):
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
        enable_processing)

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
        server.register_function(register_simulated_semantic_lidar_sensor, "register_simulated_semantic_lidar_sensor")
        server.register_function(simulated_lidar_sensor.get_detected_objects_json, "get_detected_objects_json")
        server.serve_forever()
    else:
        return simulated_lidar_sensor


def get_mock_detected_objects_json():
    class MockDetectedObject:
        def __init__(self,
                     id,
                     type,
                     x,
                     y,
                     z):
            self.id = id
            self.type = type
            self.x = x
            self.y = y
            self.z = z


    detected_objects = [
        MockDetectedObject(id=0, type="Vehicle", x=0.0, y=0.0, z=0.0),
        MockDetectedObject(id=1, type="Vehicle", x=1.0, y=1.0, z=1.0),
        MockDetectedObject(id=2, type="Vehicle", x=2.0, y=2.0, z=2.0),
        MockDetectedObject(id=3, type="Vehicle", x=3.0, y=3.0, z=3.0)
    ]

    return SimulatedSensorUtils.serialize_to_json(detected_objects)

def main_mock_data(xmlrpc_server_host, xmlrpc_server_port):
    print("Starting sensorlib XML-RPC server in mock mode.")
    server = SimpleXMLRPCServer((xmlrpc_server_host, xmlrpc_server_port))
    server.register_function(get_mock_detected_objects_json, "get_detected_objects_json")
    server.serve_forever()


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
        "--enable-processing",
        default=True,
        type=bool,
        help="Enable processing mode. Disable to bypass processing, producing only 10 detected objects using the truth state. (default: True)")

    arg_parser.add_argument(
        "--mock-data",
        default=False,
        type=bool,
        help="Enable mock data generation, bypassing the need for a CARLA connection. (default: False)")

    args = arg_parser.parse_args()

    if args.mock_data:
        main_mock_data(args.xmlrpc_server_host, args.xmlrpc_server_port)
    else:
        main(args.id, args.sensor_config, args.noise_model_config, args.detection_cycle_delay_seconds,
             args.carla_host, args.carla_port,
             args.start_rpc_server, args.xmlrpc_server_host, args.xmlrpc_server_port,
             args.enable_processing)
