#!/usr/bin/python
# -*- coding: utf-8 -*-
from src.SensorDataCollector import SensorDataCollector
from src.noise_models.NoiseModelFactory import NoiseModelFactory
from src.objects.ProxySensor import ProxySensor
from src.util.SimulatedSensorUtils import SimulatedSensorUtils


class SimulatedSensor:
    """
    Configuration steps:
    1. SimulatedSensor settings
    2. CARLA sensor settings from either:
        a) Existing spawned sensor
        b) Existing blueprint
        c) Configuration only
    3. Noise model
    """

    def __init__(self, carla_world):
        self.__carla_world = carla_world
        self.__config = None
        self.__carla_sensor = None
        self.__raw_sensor_data_collector = None
        self.__noise_model = None

        # Configuration stages required before beginning operation
        self.__is_configuration_loaded = False
        self.__is_sensor_configured = False
        self.__is_noise_model_configured = False

    # ------------------------------------------------------------------------------
    # Configuration loading
    # ------------------------------------------------------------------------------

    def configure_simulated_sensor(self, config):
        self.__config = config
        self.__is_configuration_loaded = True

    # ------------------------------------------------------------------------------
    # Sensor configuration
    # ------------------------------------------------------------------------------

    def configure_carla_sensor_from_config(self, carla_sensor_config, transform=None, parent_object=None):
        carla_sensor_blueprint = self.__carla_world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        for sensor_attribute_pair in carla_sensor_config:
            carla_sensor_blueprint.set_attribute(sensor_attribute_pair[0], sensor_attribute_pair[1])
        self.configure_carla_sensor_from_blueprint(carla_sensor_blueprint, transform, parent_object)

    def configure_carla_sensor_from_blueprint(self, carla_sensor_blueprint, transform, parent_object=None):
        carla_sensor = self.__carla_world.spawn_actor(carla_sensor_blueprint, transform, attach_to=parent_object)
        self.configure_carla_sensor_from_sensor_actor(carla_sensor)

    def configure_carla_sensor_from_sensor_actor(self, carla_sensor):
        self.__carla_sensor = carla_sensor
        self.__raw_sensor_data_collector = SensorDataCollector(self.__carla_world, carla_sensor)
        self.__is_sensor_configured = True

    # ------------------------------------------------------------------------------
    # Noise model configuration
    # ------------------------------------------------------------------------------

    def configure_noise_model(self, noise_model_config, noise_model_name="GaussianNoiseModel"):
        self.__noise_model = NoiseModelFactory.get_noise_model(noise_model_name, noise_model_config)
        self.__is_noise_model_configured = True

    # ------------------------------------------------------------------------------
    # Operation
    # ------------------------------------------------------------------------------

    def get_detected_objects_in_frame(self):
        # Ensure configuration before starting
        if not (self.__is_configuration_loaded and self.__is_sensor_configured and self.__is_noise_model_configured):
            raise Exception("SimulatedSensor must be configured before use.")

        # Get sensor including current location and configured sensor parameters
        sensor = ProxySensor(self.__carla_sensor)

        # Get detected_object truth states from simulation
        detected_objects = SimulatedSensorUtils.get_scene_detected_objects()

        # Prefilter
        detected_objects = SimulatedSensorUtils.prefilter(sensor, detected_objects)

        # Get LIDAR hitpoints with Actor ID associations
        hitpoints = self.__raw_sensor_data_collector.get_carla_lidar_hitpoints()

        # Compute data needed for occlusion operation
        actor_angular_extents = SimulatedSensorUtils.compute_actor_angular_extents(sensor, detected_objects)
        detection_thresholds = SimulatedSensorUtils.compute_adjusted_detection_thresholds(sensor, detected_objects)

        # Apply occlusion
        detected_objects = SimulatedSensorUtils.apply_occlusion(detected_objects, actor_angular_extents, hitpoints,
                                                                detection_thresholds)
        # Apply noise
        detected_objects = SimulatedSensorUtils.apply_noise(detected_objects, self.__noise_model)

        return detected_objects

    def get_detected_objects_in_frame__simple(self):
        if not (self.__is_configuration_loaded and self.__is_sensor_configured and self.__is_noise_model_configured):
            raise Exception("SimulatedSensor must be configured before use.")
        detected_objects = SimulatedSensorUtils.get_scene_detected_objects(self.__carla_world, self.__config)
        hitpoints = self.__raw_sensor_data_collector.get_carla_lidar_hitpoints()
        detected_objects = list(filter(lambda obj: obj.get_id() in hitpoints, detected_objects))
        print("Number of detected objects: ", len(detected_objects))
        return detected_objects
