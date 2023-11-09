from sensorlib.util.carla_loader import CarlaLoader
import carla
from util.simulated_sensor_utils import SimulatedSensorUtils


class IntegrationTestUtilities:

    @staticmethod
    def delete_simulation_objects(carla_world):
        actors = carla_world.get_actors()
        actors = actors.filter("vehicle*")
        for actor in actors:
            actor.destroy()

    @staticmethod
    def set_spectator_position(self, position, pitch, roll, yaw):
        transform = carla.Transform(position)
        transform.rotation.pitch = pitch
        transform.rotation.pitch = roll
        transform.rotation.pitch = yaw
        spectator = carla.get_actors().find("spectator")
        spectator.set_transform(transform)

    @staticmethod
    def create_vehicle(carla_world, position):
        blueprint_library = carla_world.get_blueprint_library()
        bp = blueprint_library.filter("model3")[0]
        transform = carla.Transform(position)
        return carla_world.spawn_actor(bp, transform)

    @staticmethod
    def create_pedestrian(carla_world, position):
        blueprint_library = carla_world.get_blueprint_library()
        pedestrian_bp = blueprint_library.filter('*walker.pedestrian*')[0]
        transform = carla.Transform(position)
        return carla_world.spawn_actor(pedestrian_bp, transform)

    @staticmethod
    def create_lidar_sensor(api, infrastructure_id, sensor_id, position, parent_id=None, custom_callback=None):
        detection_cycle_delay_seconds = 0.5
        sensor_config = SimulatedSensorUtils.load_config_from_file("config/simulated_sensor_config.yaml")
        simulated_sensor_config = sensor_config["simulated_sensor"]
        carla_sensor_config = sensor_config["lidar_sensor"]
        noise_model_config = SimulatedSensorUtils.load_config_from_file("config/noise_model_config.yaml")
        lidar_transform = carla.Transform(position)

        return api.create_simulated_semantic_lidar_sensor(simulated_sensor_config, carla_sensor_config,
                                                          noise_model_config,
                                                          detection_cycle_delay_seconds,
                                                          infrastructure_id, sensor_id,
                                                          lidar_transform.location, lidar_transform.rotation,
                                                          parent_id,
                                                          custom_callback)
