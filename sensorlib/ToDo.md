# TODO Change infrastructure ID to use sub-id (sensor ID)
# TODO Change noise model to only change a subset of the output objects.
# TODO Change noise model to toggle each noise model stage.
# TODO Clarify all sensor parameter names to indicate carla_sensor, carla_sensor_wrapper, simulated_sensor
# TODO Clarify interfaces int he readme
# TODO Clarify transform absolute/relative if attached to a parent
# TODO Collapse config from: simulated_sensor_config, carla_sensor_config, noise_model_config
# TODO Enable sensor to be stopped and started listenting
# TODO Expose configs to the RPC
# TODO Fix object serialization.
# TODO Get_detected_objects has no ID parameter
# TODO Get the xml rpc pr in place
# TODO License forall files
# TODO Make a library PR (separate)
# TODO Make the service restartable by adding a search for existing sensors, and also add passive SimulatedSensor creation from existing CARLA sensor instance.
# TODO Move sensorlib sensor to be client to any RPC connection
# TODO One RPC function to create a sensor at a given location/orientation.
# TODO One RPC function to retrieve all detected objects from the sensor.
# TODO Remove debug parameters
# TODO remove machinet.conf
# TODO Return ID or instance from the creation call in both interfaces
# TODO Sensor management is not properly exposed in the interface (multiple sensors in the RPC)
# TODO Sensor thread compute should be optional capability
# TODO Separate RPC server starting from all actions to build and run and expose the sensor capabilities.
# TODO Separate the service main.py from the library
# TODO Share carla client connection between registered sensors
# TODO Turn noise model off with a different noise_model_name in the config.
