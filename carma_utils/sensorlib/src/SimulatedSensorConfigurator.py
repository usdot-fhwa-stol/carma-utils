from src.util.SimulatedSensorUtils import SimulatedSensorUtils


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

    # ------------------------------------------------------------------------------
    # Public Interface
    # ------------------------------------------------------------------------------

    @staticmethod
    def build_simulated_sensor(world, sensor_transform, parent_actor, simulated_sensor_config_filename, noise_model_config_filename):

        # Load configurations
        combined_sensor_config = SimulatedSensorUtils.load_config_from_file(simulated_sensor_config_filename)
        simulated_sensor_config = combined_sensor_config["simulated_sensor"]
        carla_sensor_config = combined_sensor_config["lidar_sensor"]
        noise_model_config = SimulatedSensorUtils.load_config_from_file(noise_model_config_filename)

        # 
        carla_sensor = SimulatedSensorConfigurator.build_carla_semantic_lidar_sensor(world, sensor_transform, parent_actor, carla_sensor_config)
        return SimulatedSensorConfigurator.build_simulated_lidar_sensor(carla_sensor, simulated_sensor_config, noise_model_config)

    # ------------------------------------------------------------------------------
    # CARLA Sensor Building
    # ------------------------------------------------------------------------------


    @staticmethod
    def build_carla_semantic_lidar_sensor(world, sensor_transform, parent_actor, lidar_sensor_config):
        """
        Builds a CARLA Semantic LIDAR Sensor.

        :param world:
        :param sensor_transform:
        :param parent_actor:
        :return: carla_sensor
        """

        blueprint_library = world.get_blueprint_library()
        sensor_bp = SimulatedSensorConfigurator.generate_lidar_bp(world, blueprint_library, lidar_sensor_config)
        carla_sensor = world.spawn_actor(sensor_bp, sensor_transform, attach_to=parent_actor)
        return carla_sensor

    @staticmethod
    def generate_lidar_bp(world, carla_sensor_config):
        lidar_bp = world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
        lidar_bp.set_attribute("upper_fov", str(carla_sensor_config["upper_fov"]))
        lidar_bp.set_attribute("lower_fov", str(carla_sensor_config["lower_fov"]))
        lidar_bp.set_attribute("channels", str(carla_sensor_config["channels"]))
        lidar_bp.set_attribute("range", str(carla_sensor_config["range"]))
        lidar_bp.set_attribute("rotation_frequency", str(1.0 / carla_sensor_config["rotation_period"]))
        lidar_bp.set_attribute("points_per_second", str(carla_sensor_config["points_per_second"]))
        return lidar_bp

    # ------------------------------------------------------------------------------
    # Simulated Sensor Building
    # ------------------------------------------------------------------------------


    @staticmethod
    def build_simulated_lidar_sensor(carla_sensor, simulated_sensor_config, noise_model_config):
        """
        Builds a SemanticLidarSensor from a CARLA Semantic LIDAR Sensor.
        :param carla_sensor:
        :return:
        """

        sensor = CarlaSensorBuilder.build_sensor(carla_sensor)
        data_collector = SensorDataCollector(self.__carla_world, carla_sensor)
        noise_model
        
        SemanticLidarSensor(carla_sensor, data_collector, noise_model)
        
        
        
        
        simulated_sensor = SimulatedSensor(world)
        simulated_sensor.configure_simulated_sensor(SimulatedSensorUtils.load_config_from_file("../../config/simulated_sensor_config.yaml"))
        simulated_sensor.configure_carla_sensor_from_sensor_actor(lidar)
        simulated_sensor.configure_noise_model(SimulatedSensorUtils.load_config_from_file("../../config/noise_model_config.yaml"))











# ------------------------------------------------------------------------------
















def build_sensor(self):
    # Get sensor including current location and configured sensor parameters
    sensor = CarlaSensor(self.__carla_sensor)


# ------------------------------------------------------------------------------
# Configuration loading
# ------------------------------------------------------------------------------

def configure_simulated_sensor(config):
    self.__config = config
    self.__is_configuration_loaded = True


# ------------------------------------------------------------------------------
# Sensor configuration
# ------------------------------------------------------------------------------

def configure_carla_sensor_from_config(carla_sensor_config, transform=None, parent_object=None):
    carla_sensor_blueprint = self.__carla_world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
    for sensor_attribute_pair in carla_sensor_config:
        carla_sensor_blueprint.set_attribute(sensor_attribute_pair[0], sensor_attribute_pair[1])
    self.configure_carla_sensor_from_blueprint(carla_sensor_blueprint, transform, parent_object)


def configure_carla_sensor_from_blueprint(carla_sensor_blueprint, transform, parent_object=None):
    carla_sensor = self.__carla_world.spawn_actor(carla_sensor_blueprint, transform, attach_to=parent_object)
    self.configure_carla_sensor_from_sensor_actor(carla_sensor)


def configure_carla_sensor_from_sensor_actor(carla_sensor):
    self.__carla_sensor = carla_sensor
    self.__raw_sensor_data_collector = SensorDataCollector(self.__carla_world, carla_sensor)
    self.__is_sensor_configured = True


# ------------------------------------------------------------------------------
# Noise model configuration
# ------------------------------------------------------------------------------

def configure_noise_model(noise_model_config, noise_model_name="GaussianNoiseModel"):
    self.__noise_model = NoiseModelFactory.get_noise_model(noise_model_name, noise_model_config)
    self.__is_noise_model_configured = True
