# Carla CDA Sim API

The Carla CDA Sim API provides programmatic access to the library functionality. The primary means of
accessing features is through a CarlaCDASimAPI object, which may be constructed from any piece of information sufficient to
establish a Carla connection.

First ensure the following environment variables are set which define the CARLA version being utilized:

| Environment Variable | Description                                                                                                       |
| -------------------- |-------------------------------------------------------------------------------------------------------------------|
| LOAD_CARLA_EGG       | Set to True to load the CARLA Python API from an egg file. Leave blank to use a pip-managed CARLA client library. |
| CARLA_VERSION        | The CARLA version to load (ex: "0.0.14").                                                                         |
| CARLA_EGG_DIR        | (optional) Hint directory for searching for the CARLA egg file.                                                   |


Nex, build the API object.

```
api = CarlaCDASimAPI.build_from_host_spec(carla_host, carla_port)

# or...

api = CarlaCDASimAPI.build_from_client(carla_client)

# or...

api = CarlaCDASimAPI.build_from_world(carla_world)
```

The API object may then be used to indirectly build and manage sensors, and request output data. Sensors are located by
association with an infrastructure item, such as a traffic light post, and the sensor index within a co-located
collection of sensors. For example, a sensor with an infrastructure_id of 3 and sensor_id 0, is the 0th sensor
associated with infrastructure item 3.

The following sample loads configurations and constructs a sensor.

```
api = CarlaCDASimAPI.build_from_world(world)

infrastructure_id = 3
sensor_id = 7
detection_cycle_delay_seconds = 0.5
sensor_config  =  SimulatedSensorUtils.load_config_from_file(arg.sensor_config_filename)
simulated_sensor_config = sensor_config["simulated_sensor"]
carla_sensor_config = sensor_config["lidar_sensor"]
noise_model_config = SimulatedSensorUtils.load_config_from_file(arg.noise_model_config_filename)
user_offset = carla.Location(arg.x, arg.y, arg.z)
lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)

sensor = api.create_simulated_semantic_lidar_sensor(simulated_sensor_config, carla_sensor_config, noise_model_config,
                                                    detection_cycle_delay_seconds,
                                                    infrastructure_id, sensor_id,
                                                    lidar_transform.location, lidar_transform.rotation, vehicle.id)

```

Objects can be retrieved from the sensor directly:

```
detected_objects = sensor.get_detected_objects()
```

Objects can also be retrieved from another function context by using the API. (Note: Currently sensor access from the
API is only supported within the running Python context.)

```
detected_objects = api.get_detected_objects(3, 0)    # (infrastructure_id, sensor_id)
```

Previously-registered sensors can be obtained from the API.

```
sensor = api.get_simulated_sensor(3, 0)
detected_objects = sensor.get_detected_objects()
```

The CarlaCDASimAPI class provides the sensor registry and management interface, while the SimulatedLidarSensor contains the
direct sensor interface. Note that SimulatedLidarSensor is an extension of the SimulatedSensor base class, paving the
way for future sensors to be developed.

# CarlaCDASimAPI Class

## Description

Provides sensor registration, management, and access. Serves as the focal point within the library for establishing a
CARLA connection.

## Interface

    class CarlaCDASimAPI(builtins.object)
     |  Interface to build a SimulatedSensor.
     |  
     |  Methods defined here:
     |  
     |  create_simulated_semantic_lidar_sensor(self, simulated_sensor_config, carla_sensor_config, noise_model_config, detection_cycle_delay_seconds, infrastructure_id, sensor_id, sensor_position, sensor_rotation, parent_actor_id=-1)
     |      Builds a SemanticLidarSensor from a CARLA Semantic LIDAR Sensor.
     |      :param simulated_sensor_config: The configuration for the simulated sensor.
     |      :param carla_sensor_config: The configuration for the CARLA sensor.
     |      :param noise_model_config: The configuration for the noise model.
     |      :param detection_cycle_delay_seconds: The delay between sensor detections.
     |      :param infrastructure_id: The ID of the infrastructure.
     |      :param sensor_id: The ID of the sensor.
     |      :param sensor_position: Sensor position in CARLA world coordinates.
     |      :param sensor_rotation: Sensor rotation in degrees.
     |      :param parent_actor_id: ID of the parent actor to which the sensor is attached (optional).
     |      :return: A registered SimulatedSensor.
     |  
     |  get_detected_objects(self, infrastructure_id, sensor_id)
     |      Get the detected objects from a specific sensor. The DetectedObject's are those found by the sensor in the most
     |      recent call to SimulatedSensor.compute_detected_objects().
     |      
     |      :param infrastructure_id: The ID of the infrastructure.
     |      :param sensor_id: The ID of the sensor.
     |      :return: List of DetectedObject's discovered by the associated sensor.
     |  
     |  get_simulated_sensor(self, infrastructure_id, sensor_id)
     |      Get a specific simulated sensor.
     |      
     |      :param infrastructure_id: The ID of the infrastructure.
     |      :param sensor_id: The ID of the sensor.
     |      :return: The SimulatedSensor or None if not found.
     |  
     |  ----------------------------------------------------------------------
     |  Static methods defined here:
     |  
     |  build_from_client(carla_client)
     |      Build an API instance.
     |      
     |      :param carla_client: The CARLA client.
     |      :return: A CarlaCDASimAPI instance.
     |  
     |  build_from_host_spec(carla_host, carla_port)
     |      Build an API instance.
     |      
     |      :param carla_host: The CARLA host.
     |      :param carla_port: The CARLA host port.
     |      :return: A CarlaCDASimAPI instance.
     |  
     |  build_from_world(carla_world)
     |      Build an API instance.
     |      
     |      :param carla_world: The CARLA world.
     |      :return: A CarlaCDASimAPI instance.

## File

    CarlaCDASimAPI.py

# SemanticLidarSensor Class

## Description

SemanticLidarSensor is a concrete sensor extending the SimulatedSensor interface.

A call to `compute_detected_objects()` forces computation of the processing pipeline, populating an internal cache of
DetectedObject's. These are immediately returned, and available subsequently through the `get_detected_objects()`
function.

Proper construction and thread initialization is handled by the CarlaCDASimAPI, which is the recommended sensor building
interface. If constructed directly, `compute_detected_objects()` must be configured to be called repeatedly at the
desired detection computation cadence. Note that this is a blocking function.

## Interface

    class SemanticLidarSensor(SimulatedSensor)
     |  SemanticLidarSensor(infrastructure_id, sensor_id, simulated_sensor_config, carla_sensor_config, carla_world, sensor, data_collector, noise_model)
     |  
     |  Wrapper for the CARLA Semantic Lidar Sensor, with additional post-processing logic to compute a list of
     |  detected objects in the scene.
     |  
     |  Data is reported as the carla.SemanticLidarMeasurement type through the callback registered to
     |  carla.Sensor.listen(self, callback):
     |  
     |  https://carla.readthedocs.io/en/latest/python_api/#carla.Sensor.listen
     |  
     |  Methods defined here:
     |  
     |  __init__(self, infrastructure_id, sensor_id, simulated_sensor_config, carla_sensor_config, carla_world, sensor, data_collector, noise_model)
     |      Constructor.
     |      
     |      :param infrastructure_id: Infrastructure ID with which the sensor is associated.
     |      :param sensor_id: Sensor ID.
     |      :param simulated_sensor_config: Configuration dictionary containing parameters specific to the wrapped sensor logic.
     |      :param carla_sensor_config: Parameters relevant to the CARLA sensor configuration.
     |      :param carla_world: Reference to the CARLA world object.
     |      :param sensor: CarlaSensor object wrapping the CARLA sensor actor.
     |      :param data_collector: DataCollector object handling collection and caching of raw sensor data from the CARLA simulation.
     |      :param noise_model: Noise model available to be used for nosie application to the output data.
     |  
     |  compute_detected_objects(self)
     |      Main function used to query the currently-detected objects. Upon calling, the latest raw data cache is
     |      retrieved and sent through the processing pipeline to produce a list of DetectedObject objects.
     |      
     |      :return: List of DetectedObject objects serialized in JSON form.
     |  
     |  get_detected_objects(self)
     |      Returns the latest detected objects.
     |  
     |  ----------------------------------------------------------------------
     |  Methods inherited from sensor.SimulatedSensor.SimulatedSensor:
     |  
     |  get_id(self)
     |  
     |  get_infrastructure_id(self)

# File

    sensor/SemanticLidarSensor.py
