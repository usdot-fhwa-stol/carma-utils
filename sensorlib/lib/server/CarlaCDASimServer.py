from datetime import datetime

from util.SimulatedSensorUtils import SimulatedSensorUtils


class CarlaCDASimServer:
    """
    Provides core API functionality, wrapped in a servable interface.
    """

    def __init__(self, sensor_api, sensor_config, noise_model_config, detection_cycle_delay_seconds):
        self.__api = sensor_api
        self.__sensor_config = sensor_config
        self.__noise_model_config = noise_model_config
        self.__detection_cycle_delay_seconds = detection_cycle_delay_seconds

    # ------------------------------------------------------------------------------
    # Configurations
    # ------------------------------------------------------------------------------

    def set_sensor_configuration(self, sensor_config):
        self.__sensor_config = sensor_config

    def set_noise_model_configuration(self, noise_model_config):
        self.__noise_model_config = noise_model_config

    def set_detection_cycle_delay_seconds(self, detection_cycle_delay_seconds):
        self.__detection_cycle_delay_seconds = detection_cycle_delay_seconds

    # ------------------------------------------------------------------------------
    # Sensor Creation
    # ------------------------------------------------------------------------------

    def create_simulated_semantic_lidar_sensor(self, infrastructure_id, sensor_id, sensor_position, sensor_rotation):
        simulated_sensor = self.__api.create_simulated_semantic_lidar_sensor(self.__sensor_config["simulated_sensor"],
                                                                             self.__sensor_config["lidar_sensor"],
                                                                             self.__noise_model_config,
                                                                             self.__detection_cycle_delay_seconds,
                                                                             infrastructure_id, sensor_id,
                                                                             sensor_position, sensor_rotation)
        return str(simulated_sensor.get_id())

    # ------------------------------------------------------------------------------
    # Sensor and Data Access
    # ------------------------------------------------------------------------------

    def get_simulated_sensor(self, infrastructure_id, sensor_id):
        sensor = self.__api.get_simulated_sensor(infrastructure_id, sensor_id)
        return str(sensor.get_id())

    def get_detected_objects(self, infrastructure_id, sensor_id):
        detected_objects = self.__api.get_detected_objects(infrastructure_id, sensor_id)
        return_json = str(SimulatedSensorUtils.serialize_to_json(detected_objects))
        return return_json

    def get_echo_response(self):
        return f"Echo response from sensorlib XML-RPC server at {datetime.now().isoformat()}"
