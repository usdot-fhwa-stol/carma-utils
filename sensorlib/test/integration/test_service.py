# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import src

from util.CarlaLoader import CarlaLoader
import carla
import time

import xmlrpc.client

from CarlaCDASimAPI import CarlaCDASimAPI
from CarlaCDASimAdapter import CarlaCDASimAdapter
from util.SimulatedSensorUtils import SimulatedSensorUtils

# Build the connection objects
api = CarlaCDASimAPI.build_from_host_spec("localhost", 2000)
adapter = CarlaCDASimAdapter(api)

# Start the server
adapter.start_xml_rpc_server("localhost", 8000, False)

# Specify sensor parameters
infrastructure_id = 3
sensor_id = 7
detection_cycle_delay_seconds = 0.5
sensor_config_filename = "config/simulated_sensor_config.yaml"
noise_model_config_filename = "config/noise_model_config.yaml"
sensor_config = SimulatedSensorUtils.load_config_from_file(sensor_config_filename)
simulated_sensor_config = sensor_config["simulated_sensor"]
carla_sensor_config = sensor_config["lidar_sensor"]
noise_model_config = SimulatedSensorUtils.load_config_from_file(noise_model_config_filename)
user_offset = carla.Location(0.0, 0.0, 0.0)
lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)

with xmlrpc.client.ServerProxy("http://localhost:8000/") as rpc_client:

    # Build a sensor
    sensor_id = rpc_client.create_simulated_semantic_lidar_sensor(simulated_sensor_config, carla_sensor_config, noise_model_config,
                                                      detection_cycle_delay_seconds,
                                                      infrastructure_id, sensor_id,
                                                      lidar_transform.location, lidar_transform.rotation, -1)
    print(f"Created sensor {sensor_id}")

    # Retrieve the detected objects
    while True:
        time.sleep(0.8)
        detected_objects = rpc_client.get_detected_objects(infrastructure_id, sensor_id)
        print("---- Detected Objects ----")
        print(detected_objects)
        print()
