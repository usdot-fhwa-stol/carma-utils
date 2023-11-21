# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import sched
import threading
import time

from util.CarlaLoader import CarlaLoader
CarlaLoader.load_carla_lib()
import carla
import sys
sys.path.append('../')

from collector.SensorDataCollector import SensorDataCollector
from noise_models.NoiseModelFactory import NoiseModelFactory
from objects.CarlaSensor import CarlaSensorBuilder
from sensor.SemanticLidarSensor import SemanticLidarSensor
from util.CarlaUtils import CarlaUtils


class CarlaCDASimAPI:
    """
    Interface to build a SimulatedSensor.
    """

    def __init__(self):
        self.__client = None
        self.__carla_world = None
        self.__infrastructure_sensors = {}

    @staticmethod
    def build_from_host_spec(carla_host, carla_port):
        print(f"connecting to carla {carla_host}:{carla_port}")
        """
        Build an API instance.

        :param carla_host: The CARLA host.
        :param carla_port: The CARLA host port.
        :return: A CarlaCDASimAPI instance.
        """
        client = carla.Client(str(carla_host), int(carla_port))
        return CarlaCDASimAPI.build_from_client(client)

    @staticmethod
    def build_from_client(carla_client):
        """
        Build an API instance.

        :param carla_client: The CARLA client.
        :return: A CarlaCDASimAPI instance.
        """
        api = CarlaCDASimAPI()
        api.__client = carla_client
        api.__client.set_timeout(2.0)
        api.__carla_world = api.__client.get_world()
        return api

    @staticmethod
    def build_from_world(carla_world):
        """
        Build an API instance.

        :param carla_world: The CARLA world.
        :return: A CarlaCDASimAPI instance.
        """
        api = CarlaCDASimAPI()
        api.__carla_world = carla_world
        return api

    # ------------------------------------------------------------------------------
    # SimulatedSensor Management Interface
    # ------------------------------------------------------------------------------

    def create_simulated_semantic_lidar_sensor(self, simulated_sensor_config, carla_sensor_config, noise_model_config,
                                               detection_cycle_delay_seconds,
                                               infrastructure_id, sensor_id,
                                               sensor_position, sensor_rotation, parent_actor_id=-1):
        """
        Builds a SemanticLidarSensor from a CARLA Semantic LIDAR Sensor.
        :param simulated_sensor_config: The configuration for the simulated sensor.
        :param carla_sensor_config: The configuration for the CARLA sensor.
        :param noise_model_config: The configuration for the noise model.
        :param detection_cycle_delay_seconds: The delay between sensor detections.
        :param infrastructure_id: The ID of the infrastructure.
        :param sensor_id: The ID of the sensor.
        :param sensor_position: Sensor position in CARLA world coordinates.
        :param sensor_rotation: Sensor rotation in degrees.
        :param parent_actor_id: ID of the parent actor to which the sensor is attached (optional).
        :return: A registered SimulatedSensor.
        """

        # Parameter checks
        if not isinstance(infrastructure_id, int) or infrastructure_id < 0:
            print("Error: infrastructure_id needs to be a non-negative integer.")
            return None
        if not isinstance(sensor_id, int) or sensor_id < 0:
            print("Error: sensor_id needs to be a non-negative integer.")
            return None

        # Build the transform
        sensor_transform = CarlaUtils.get_transform(sensor_position, sensor_rotation)

        # Retrieve the CARLA sensor
        blueprint_library = self.__carla_world.get_blueprint_library()
        sensor_bp = self.__generate_lidar_bp(blueprint_library, carla_sensor_config)
        parent_actor = CarlaUtils.get_actor(self.__carla_world, parent_actor_id)
        carla_sensor = self.__carla_world.spawn_actor(sensor_bp, sensor_transform, attach_to=parent_actor)
        self.__carla_world.wait_for_tick()

        # Build internal objects
        sensor = CarlaSensorBuilder.build_sensor(carla_sensor)
        data_collector = SensorDataCollector(self.__carla_world, carla_sensor)
        noise_model = NoiseModelFactory.get_noise_model(noise_model_config["noise_model_name"], noise_model_config)

        # Construct the SimulatedSensor
        simulated_sensor = SemanticLidarSensor(infrastructure_id, sensor_id, simulated_sensor_config,
                                               carla_sensor_config,
                                               self.__carla_world, sensor,
                                               data_collector, noise_model)

        # Register the sensor
        self.__infrastructure_sensors[(infrastructure_id, sensor_id)] = simulated_sensor

        # Start compute thread
        scheduler = sched.scheduler(time.time, time.sleep)
        scheduler.enter(detection_cycle_delay_seconds, 1, self.__schedule_next_compute,
                        (scheduler, simulated_sensor, detection_cycle_delay_seconds))
        scheduler_thread = threading.Thread(target=scheduler.run)
        print("Starting sensorlib compute.")
        scheduler_thread.start()

        return simulated_sensor

    def get_simulated_sensor(self, infrastructure_id, sensor_id):
        """
        Get a specific simulated sensor.

        :param infrastructure_id: The ID of the infrastructure.
        :param sensor_id: The ID of the sensor.
        :return: The SimulatedSensor or None if not found.
        """
        return self.__infrastructure_sensors.get((infrastructure_id, sensor_id))

    def get_detected_objects(self, infrastructure_id, sensor_id):
        """
        Get the detected objects from a specific sensor. The DetectedObject's are those found by the sensor in the most
        recent call to SimulatedSensor.compute_detected_objects().

        :param infrastructure_id: The ID of the infrastructure.
        :param sensor_id: The ID of the sensor.
        :return: List of DetectedObject's discovered by the associated sensor.
        """
        simulated_sensor = self.__infrastructure_sensors.get((infrastructure_id, sensor_id))
        return simulated_sensor.get_detected_objects()

    # ------------------------------------------------------------------------------
    # Helper Functions
    # ------------------------------------------------------------------------------

    def __schedule_next_compute(self, scheduler, simulated_sensor, detection_cycle_delay_seconds):
        """Schedule the next compute cycle."""
        scheduler.enter(detection_cycle_delay_seconds, 1, self.__schedule_next_compute,
                        (scheduler, simulated_sensor, detection_cycle_delay_seconds))
        simulated_sensor.compute_detected_objects()

    def __generate_lidar_bp(self, blueprint_library, carla_sensor_config):
        """Build the CARLA blueprint necessary for CARLA sensor construction."""
        lidar_bp = blueprint_library.find("sensor.lidar.ray_cast_semantic")
        lidar_bp.set_attribute("upper_fov", str(carla_sensor_config["upper_fov"]))
        lidar_bp.set_attribute("lower_fov", str(carla_sensor_config["lower_fov"]))
        lidar_bp.set_attribute("channels", str(carla_sensor_config["channels"]))
        lidar_bp.set_attribute("range", str(carla_sensor_config["range"]))
        lidar_bp.set_attribute("rotation_frequency", str(1.0 / carla_sensor_config["rotation_period"]))
        lidar_bp.set_attribute("points_per_second", str(carla_sensor_config["points_per_second"]))
        return lidar_bp
