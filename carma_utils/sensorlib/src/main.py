# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import argparse
import json

import sched
import time
from xmlrpc.server import SimpleXMLRPCServer
import threading

import carla

from src.SimulatedSensorConfigurator import SimulatedSensorConfigurator
from src.util.SimulatedSensorUtils import SimulatedSensorUtils

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
        "--detection-cycle-delay-seconds",
        default=0.5,
        type=float,
        help="Delay in continuous sensor processing loop in seconds. (default: 0.5)")

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

    # Get inputs
    infrastructure_id = args.id
    detection_cycle_delay_seconds = args.detection_cycle_delay_seconds
    sensor_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0),
                                       carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))

    # Build configuration
    if args.sensor_config == "":
        config = SimulatedSensorUtils.load_config_from_file("../config/simulated_sensor_config.yaml")
    else:
        config = json.loads(args.sensor_config)
    simulated_sensor_config = config["simulated_sensor"]
    carla_sensor_config = config["lidar_sensor"]

    if args.noise_model_config == "":
        noise_model_config = SimulatedSensorUtils.load_config_from_file("../config/noise_model_config.yaml")
    else:
        noise_model_config = json.loads(args.noise_model_config)

    # Build sensor
    # simulated_lidar_sensor = SimulatedSensorConfigurator.register_simulated_semantic_lidar_sensor(
    #     simulated_sensor_config,
    #     carla_sensor_config,
    #     noise_model_config,
    #     infrastructure_id,
    #     sensor_transform,
    #     None)
    class A:
        def compute_detected_objects(self):
            print("compute_detected_objects")
        def get_detected_objects_json(self):
            return ""
    simulated_lidar_sensor = A()

    # Compute detected objects continuously using a separate thread
    scheduler = sched.scheduler(time.time, time.sleep)
    detection_cycle_delay_seconds = 1
    def scheduled_compute():
        scheduler.enter(detection_cycle_delay_seconds, 1, scheduled_compute)
        simulated_lidar_sensor.compute_detected_objects()

    scheduler.enter(detection_cycle_delay_seconds, 1, scheduled_compute)
    scheduler_thread = threading.Thread(target=scheduler.run)
    scheduler_thread.start()

    # Create an XML-RPC server
    print("starting rpc server")
    server = SimpleXMLRPCServer((args.xmlrpc_server_host, args.xmlrpc_server_port))
    server.register_function(simulated_lidar_sensor.get_detected_objects_json, "periodic_function")
    server.serve_forever()
