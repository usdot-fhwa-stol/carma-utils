from datetime import time

from src.SimulatedSensorConfigurator import SimulatedSensorConfigurator
from src.util.SimulatedSensorUtils import SimulatedSensorUtils

if __name__ == "__main__":

    # Build sensor
    config = SimulatedSensorUtils.load_config_from_file("simulated_sensor_config.yaml")
    simulated_sensor_config = config["simulated_sensor"]
    carla_sensor_config = config["lidar_sensor"]
    noise_model_config = SimulatedSensorUtils.load_config_from_file("nosie_model_config.yaml")

simulated_lidar_sensor = SimulatedSensorConfigurator.register_simulated_semantic_lidar_sensor(simulated_sensor_config, carla_sensor_config, noise_model_config,
                                                                                                  infrastructure_id, sensor_transform, None)

    # Compute detected objects continuously
    while True:
        simulated_lidar_sensor.compute_detected_objects()
        time.sleep(0.1)
