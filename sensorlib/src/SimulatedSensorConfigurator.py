# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

# from util.CarlaLoader import CarlaLoader
# CarlaLoader.load_carla_lib("0.9.14")
import carla
import numpy as np

from SemanticLidarSensor import SemanticLidarSensor
from collector.SensorDataCollector import SensorDataCollector
from noise_models.NoiseModelFactory import NoiseModelFactory
from objects.CarlaSensor import CarlaSensorBuilder


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
                                                 carla_host, carla_port,
                                                 sensor_transform, infrastructure_id=-1, parent_actor=None,
                                                 enable_processing=True):
        """
        Builds a SemanticLidarSensor from a CARLA Semantic LIDAR Sensor.
        :param simulated_sensor_config: The configuration for the simulated sensor.
        :param carla_sensor_config: The configuration for the CARLA sensor.
        :param noise_model_config: The configuration for the noise model.
        :param carla_host: The CARLA host.
        :param carla_port: The CARLA host port.
        :param sensor_transform: The transform of the sensor.
        :param infrastructure_id: The ID of the infrastructure. Any existing sensor with this ID is overwritten
                        in the registry, but references to the sensor are not invalidated. Negative value or
                        None forces auto-assignment.
        :param parent_actor: The parent actor of the sensor (optional).
        :return: A registered SemanticLidarSensor.
        """

        carla_world = SimulatedSensorConfigurator.__get_initialized_carla_world(carla_host, carla_port)

        # Retrieve the CARLA sensor
        blueprint_library = carla_world.get_blueprint_library()
        sensor_bp = SimulatedSensorConfigurator.__generate_lidar_bp(blueprint_library, carla_sensor_config)
        carla_sensor = carla_world.spawn_actor(sensor_bp, sensor_transform, attach_to=parent_actor)

        # Build internal objects
        sensor = CarlaSensorBuilder.build_sensor(carla_sensor)
        data_collector = SensorDataCollector(carla_world, carla_sensor, enable_processing)
        noise_model = NoiseModelFactory.get_noise_model(noise_model_config["noise_model_name"], noise_model_config)

        # Determine the infrastructure ID
        if infrastructure_id is None or infrastructure_id < 0:
            if len(SimulatedSensorConfigurator.__infrastructure_sensors) == 0:
                infrastructure_id = 0
            else:
                infrastructure_id = np.max(list(SimulatedSensorConfigurator.__infrastructure_sensors.keys())) + 1

        # Construct the SimulatedSensor
        simulated_sensor = SemanticLidarSensor(infrastructure_id, simulated_sensor_config, carla_sensor_config,
                                               carla_world, sensor,
                                               data_collector, noise_model, enable_processing)

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
    def __get_initialized_carla_world(carla_host, carla_port):
        """Initialize the CARLA connection, which only needs to be done once."""

        if SimulatedSensorConfigurator.__client is None:
            SimulatedSensorConfigurator.__client = carla.Client("localhost", 2000)
            # SimulatedSensorConfigurator.__client = carla.Client(carla_host, carla_port)
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
