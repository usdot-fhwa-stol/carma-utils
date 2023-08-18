from src.SemanticLidarSensor import SemanticLidarSensor
from src.collector.SensorDataCollector import SensorDataCollector
from src.noise_models.NoiseModelFactory import NoiseModelFactory
from src.objects.CarlaSensor import CarlaSensorBuilder
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

    # TODO load config from file

    # TODO Function to build and register a sensor
    #  def register_sensor(sensor_position, sensor_orientation, [sensor_transform],
    #  config dictionary)

    # TODO Retrieve data function
    ## TODO simplify this to require no world, a transform, optional parentobejct, infrasture ID (store it inside an attributes dictionary or something, and possibly keep up-to-date DetectedObject cache), simulate
    ## TODO return JSON data instead

    # TODO Retrieve sensor by infrastructure ID


    @staticmethod
    def build_simulated_sensor(carla_world, sensor_transform, parent_actor, simulated_sensor_config_filename,
                               noise_model_config_filename):
        # Load configurations
        combined_sensor_config = SimulatedSensorUtils.load_config_from_file(simulated_sensor_config_filename)
        simulated_sensor_config = combined_sensor_config["simulated_sensor"]
        carla_sensor_config = combined_sensor_config["lidar_sensor"]
        noise_model_config = SimulatedSensorUtils.load_config_from_file(noise_model_config_filename)

        # Build the sensor
        carla_sensor = SimulatedSensorConfigurator.build_carla_semantic_lidar_sensor(carla_world, sensor_transform,
                                                                                     parent_actor, carla_sensor_config)
        return SimulatedSensorConfigurator.build_simulated_lidar_sensor(carla_world, carla_sensor,
                                                                        simulated_sensor_config, carla_sensor_config,
                                                                        noise_model_config)

    # ------------------------------------------------------------------------------
    # CARLA Sensor Building
    # ------------------------------------------------------------------------------

    @staticmethod
    def build_carla_semantic_lidar_sensor(carla_world, sensor_transform, parent_actor, lidar_sensor_config):
        """
        Builds a CARLA Semantic LIDAR Sensor.

        :param carla_world:
        :param sensor_transform:
        :param parent_actor:
        :return: carla_sensor
        """

        blueprint_library = carla_world.get_blueprint_library()
        sensor_bp = SimulatedSensorConfigurator.generate_lidar_bp(blueprint_library, lidar_sensor_config)
        carla_sensor = carla_world.spawn_actor(sensor_bp, sensor_transform, attach_to=parent_actor)
        return carla_sensor

    @staticmethod
    def generate_lidar_bp(blueprint_library, carla_sensor_config):
        lidar_bp = blueprint_library.find("sensor.lidar.ray_cast_semantic")
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
    def build_simulated_lidar_sensor(carla_world, carla_sensor, simulated_sensor_config, carla_sensor_config,
                                     noise_model_config):
        """
        Builds a SemanticLidarSensor from a CARLA Semantic LIDAR Sensor.
        :param carla_sensor:
        :return:
        """

        # Build internal objects
        sensor = CarlaSensorBuilder.build_sensor(carla_sensor)
        data_collector = SensorDataCollector(carla_world, carla_sensor)
        noise_model = NoiseModelFactory.get_noise_model(noise_model_config["noise_model_name"], noise_model_config)

        # Construct the SimulatedSensor
        return SemanticLidarSensor(simulated_sensor_config, carla_sensor_config, carla_world, sensor, data_collector,
                                   noise_model)
