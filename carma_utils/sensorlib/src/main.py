import argparse
import json
import time

import carla

from src.SimulatedSensorConfigurator import SimulatedSensorConfigurator
from src.util.SimulatedSensorUtils import SimulatedSensorUtils

if __name__ == "__main__":

    # Parse arguments
    arg_parser = argparse.ArgumentParser(
        description=__doc__)

    arg_parser.add_argument(
        "--id",
        default=-1,
        type=int,
        help="Infrastructure ID to assign to the sensor. Negative value forces auto-assignment. (default: -1)")

    arg_parser.add_argument(
        "--detection-cycle-delay-seconds",
        default=0.5,
        type=float,
        help="Delay in continuous sensor processing loop in seconds. (default: 0.5)")

    arg_parser.add_argument(
        "--sensor-config",
        default="",
        type=str,
        help="Sensor configuration as JSON dictionary. (default: \"\")")

    arg_parser.add_argument(
        "--noise-model-config",
        default="",
        type=str,
        help="Noise model configuration as JSON dictionary. (default: \"\")")

    args = arg_parser.parse_args()

    # Get inputs
    infrastructure_id = args.id
    detection_cycle_delay_seconds = args.detection_cycle_delay_seconds
    sensor_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0),
                                       carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))

    # Build configuration
    if args.sensor_config == "":
        config = SimulatedSensorUtils.load_config_from_file("../config/simulated_sensor_config.yaml")
    else:
        config = json.loads(args.sensor_config)
    simulated_sensor_config = config["simulated_sensor"]
    carla_sensor_config = config["lidar_sensor"]

    if args.noise_model_config == "":
        noise_model_config = SimulatedSensorUtils.load_config_from_file("../config/noise_model_config.yaml")
    else:
        noise_model_config = json.loads(args.noise_model_config)

    # Build sensor
    simulated_lidar_sensor = SimulatedSensorConfigurator.register_simulated_semantic_lidar_sensor(
        simulated_sensor_config,
        carla_sensor_config,
        noise_model_config,
        infrastructure_id,
        sensor_transform,
        None)

    # Compute detected objects continuously
    while True:
        simulated_lidar_sensor.compute_detected_objects()
        time.sleep(detection_cycle_delay_seconds)
