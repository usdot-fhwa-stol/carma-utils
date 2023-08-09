class SimulatedSensorConfigurator:
    """
    Configuration steps:
    1. SimulatedSensor settings
    2. CARLA sensor settings from either:
        a) Existing spawned sensor
        b) Existing blueprint
        c) Configuration only
    3. Noise model
    """



    # self.__carla_world = carla_world
    # self.__config = None
    # self.__raw_carla_sensor = None




    def build_sensor(self):
        # Get sensor including current location and configured sensor parameters
        sensor = CarlaSensor(self.__carla_sensor)









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
