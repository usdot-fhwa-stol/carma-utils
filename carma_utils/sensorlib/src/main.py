from datetime import time

from src.SimulatedSensorConfigurator import SimulatedSensorConfigurator

if __name__ == "__main__":

    # Build sensor
    simulated_lidar_sensor = SimulatedSensorConfigurator.register_simulated_semantic_lidar_sensor()

    # Compute detected objects continuously
    while True:
        simulated_lidar_sensor.compute_detected_objects()
        time.sleep(0.1)
