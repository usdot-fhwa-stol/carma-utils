# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
import carla

from src.SemanticLidarSensor import SemanticLidarSensor
from src.collector.SensorDataCollector import SensorDataCollector
from src.noise_models.NoiseModelFactory import NoiseModelFactory
from src.objects.CarlaSensor import CarlaSensorBuilder
from src.util.SimulatedSensorUtils import SimulatedSensorUtils


class SimulatedSensorConfigurator:
    """
    Interface to build a SimulatedSensor.

    Configuration steps:
    1. SimulatedSensor settings
    2. CARLA sensor settings from either:
        a) Existing spawned sensor
        b) Existing blueprint
        c) Configuration only
    3. Noise model
    """

    def __init__(self):
        SimulatedSensorConfigurator.infrastructure_sensors = {}
        SimulatedSensorConfigurator.client = None
        SimulatedSensorConfigurator.carla_world = None

    # TODO load config from file

    # TODO Function to build and register a sensor
    #  def register_sensor(sensor_position, sensor_orientation, [sensor_transform],
    #  config dictionary)

    # TODO Retrieve data function
    ## TODO simplify this to require no world, a transform, optional parentobejct, infrasture ID (store it inside an attributes dictionary or something, and possibly keep up-to-date DetectedObject cache), simulate

    @staticmethod
    def build_simulated_semantic_lidar_sensor(simulated_sensor_config, carla_sensor_config,
                                              noise_model_config,
                                              infrastructure_id, transform, parent_object=None):
        """
        Builds a SemanticLidarSensor from a CARLA Semantic LIDAR Sensor.
        :return:
        """

        # Initialize the CARLA connection, which only needs to be done once
        if SimulatedSensorConfigurator.client is None:
            SimulatedSensorConfigurator.client = carla.Client(simulated_sensor_config["carla_connection"]["carla_host"],
                                                              simulated_sensor_config["carla_connection"]["carla_port"])
            SimulatedSensorConfigurator.client.set_timeout(2.0)
            SimulatedSensorConfigurator.carla_world = SimulatedSensorConfigurator.client.get_world()

        carla_world = SimulatedSensorConfigurator.carla_world

        # Retrieve the CARLA sensor
        carla_sensor = None

        # Build internal objects
        sensor = CarlaSensorBuilder.build_sensor(carla_sensor)
        data_collector = SensorDataCollector(carla_world, carla_sensor)
        noise_model = NoiseModelFactory.get_noise_model(noise_model_config["noise_model_name"], noise_model_config)

        # Construct the SimulatedSensor
        simulated_sensor = SemanticLidarSensor(infrastructure_id, simulated_sensor_config, carla_sensor_config,
                                               carla_world, sensor,
                                               data_collector, noise_model)
        SimulatedSensorConfigurator.infrastructure_sensors[infrastructure_id] = simulated_sensor
        return SemanticLidarSensor(simulated_sensor_config, carla_sensor_config, carla_world, sensor, data_collector,
                                   noise_model)

    @staticmethod
    def get_simulated_sensor(infrastructure_id):
        return SimulatedSensorConfigurator.infrastructure_sensors.get(infrastructure_id)
