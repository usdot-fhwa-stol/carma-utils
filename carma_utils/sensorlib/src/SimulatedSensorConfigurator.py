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
    """

    # Static fields
    __infrastructure_sensors = {}
    __client = None
    __carla_world = None

    # ------------------------------------------------------------------------------
    # SimulatedSensor Management Interface
    # ------------------------------------------------------------------------------

    @staticmethod
    def register_simulated_semantic_lidar_sensor(simulated_sensor_config, carla_sensor_config, noise_model_config,
                                                 infrastructure_id, sensor_transform, parent_actor=None):
        """
        Builds a SemanticLidarSensor from a CARLA Semantic LIDAR Sensor.
        :param simulated_sensor_config: The configuration for the simulated sensor.
        :param carla_sensor_config: The configuration for the CARLA sensor.
        :param noise_model_config: The configuration for the noise model.
        :param infrastructure_id: The ID of the infrastructure.
        :param sensor_transform: The transform of the sensor.
        :param parent_actor: The parent actor of the sensor (optional).
        :return: A registered SemanticLidarSensor.
        """

        carla_world = SimulatedSensorConfigurator.__get_initialized_carla_world(simulated_sensor_config)

        # Retrieve the CARLA sensor
        blueprint_library = carla_world.get_blueprint_library()
        sensor_bp = SimulatedSensorConfigurator.__generate_lidar_bp(blueprint_library, carla_sensor_config)
        carla_sensor = carla_world.spawn_actor(sensor_bp, sensor_transform, attach_to=parent_actor)

        # Build internal objects
        sensor = CarlaSensorBuilder.build_sensor(carla_sensor)
        data_collector = SensorDataCollector(carla_world, carla_sensor)
        noise_model = NoiseModelFactory.get_noise_model(noise_model_config["noise_model_name"], noise_model_config)

        # Construct the SimulatedSensor
        simulated_sensor = SemanticLidarSensor(infrastructure_id, simulated_sensor_config, carla_sensor_config,
                                               carla_world, sensor,
                                               data_collector, noise_model)

        # Register the sensor for fast retrieval
        SimulatedSensorConfigurator.__infrastructure_sensors[infrastructure_id] = simulated_sensor

        return simulated_sensor

    @staticmethod
    def get_simulated_sensor(infrastructure_id):
        """Retrieve a SimulatedSensor."""
        return SimulatedSensorConfigurator.__infrastructure_sensors.get(infrastructure_id)

    # ------------------------------------------------------------------------------
    # Helper Functions
    # ------------------------------------------------------------------------------

    @staticmethod
    def __get_initialized_carla_world(simulated_sensor_config):
        """Initialize the CARLA connection, which only needs to be done once."""

        if SimulatedSensorConfigurator.__client is None:
            SimulatedSensorConfigurator.__client = carla.Client(
                simulated_sensor_config["carla_connection"]["carla_host"],
                simulated_sensor_config["carla_connection"]["carla_port"])
            SimulatedSensorConfigurator.__client.set_timeout(2.0)
            SimulatedSensorConfigurator.__carla_world = SimulatedSensorConfigurator.__client.get_world()

        return SimulatedSensorConfigurator.__carla_world

    @staticmethod
    def __generate_lidar_bp(blueprint_library, carla_sensor_config):
        """Build the CARLA blueprint necessary for CARLA sensor construction."""
        lidar_bp = blueprint_library.find("sensor.lidar.ray_cast_semantic")
        lidar_bp.set_attribute("upper_fov", str(carla_sensor_config["upper_fov"]))
        lidar_bp.set_attribute("lower_fov", str(carla_sensor_config["lower_fov"]))
        lidar_bp.set_attribute("channels", str(carla_sensor_config["channels"]))
        lidar_bp.set_attribute("range", str(carla_sensor_config["range"]))
        lidar_bp.set_attribute("rotation_frequency", str(1.0 / carla_sensor_config["rotation_period"]))
        lidar_bp.set_attribute("points_per_second", str(carla_sensor_config["points_per_second"]))
        return lidar_bp
