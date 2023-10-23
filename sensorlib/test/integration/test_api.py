# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

from src.util.CarlaLoader import CarlaLoader
import carla

import time

from src.CarlaCDASimAPI import CarlaCDASimAPI
from src.util.SimulatedSensorUtils import SimulatedSensorUtils

# Build the connection objects
api = CarlaCDASimAPI.build_from_host_spec("localhost", 2000)

# Specify sensor parameters
infrastructure_id = 3
sensor_id = 7
detection_cycle_delay_seconds = 0.5
sensor_config_filename = "../../config/simulated_sensor_config.yaml"
noise_model_config_filename = "../../config/noise_model_config.yaml"
sensor_config = SimulatedSensorUtils.load_config_from_file(sensor_config_filename)
simulated_sensor_config = sensor_config["simulated_sensor"]
carla_sensor_config = sensor_config["lidar_sensor"]
noise_model_config = SimulatedSensorUtils.load_config_from_file(noise_model_config_filename)
user_offset = carla.Location(0.0, 0.0, 0.0)
lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)

# Build a sensor
sensor = api.create_simulated_semantic_lidar_sensor(simulated_sensor_config, carla_sensor_config, noise_model_config,
                                                    detection_cycle_delay_seconds,
                                                    infrastructure_id, sensor_id,
                                                    lidar_transform.location, lidar_transform.rotation, -1)

# Validate objects are detected
time.sleep(1.2)
detected_objects = api.get_detected_objects(infrastructure_id, sensor_id)
assert len(detected_objects) > 0, "No objects detected"
print("---- Detected Objects ----")
print(detected_objects)
print()
