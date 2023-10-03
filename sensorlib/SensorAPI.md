
The Python API provides programmatic access to the library functionality.


NAME
    SensorAPI

DESCRIPTION

CLASSES
    
    class SensorAPI
     |  Interface to build a SimulatedSensor.
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
     |      :return: A registered SemanticLidarSensor.
     |  
     |  get_detected_objects(self, infrastructure_id, sensor_id)
     |      Get the detected objects from a specific sensor.
     |  
     |  get_simulated_sensor(self, infrastructure_id, sensor_id)
     |      Get a specific simulated sensor.
     |  
     |  ----------------------------------------------------------------------
     |  Static methods defined here:
     |  
     |  build_from_client(carla_client)
     |      Build an API instance.
     |      
     |      :param carla_client: The CARLA client.
     |      :return: A SensorAPI instance.
     |  
     |  build_from_host_spec(carla_host, carla_port)
     |      Build an API instance.
     |      
     |      :param carla_host: The CARLA host.
     |      :param carla_port: The CARLA host port.
     |      :return: A SensorAPI instance.
     |  
     |  build_from_world(carla_world)
     |      Build an API instance.
     |      
     |      :param carla_world: The CARLA world.
     |      :return: A SensorAPI instance.
     |  
     |  ----------------------------------------------------------------------
     |  Data descriptors defined here:
     |  
     |  __dict__
     |      dictionary for instance variables (if defined)
     |  
     |  __weakref__
     |      list of weak references to the object (if defined)

FILE
    /home/david/code/stol/sim/carma-utils-1/sensorlib/SensorAPI.py



















### create_simulated_semantic_lidar_sensor

#### Input Parameters


| Parameter                     | Type    | Description                                                                         |
|-------------------------------|---------|-------------------------------------------------------------------------------------|
| simulated_sensor_config       | JSON    | The configuration for the simulated sensor.                                         |
| carla_sensor_config           | JSON    | The configuration for the CARLA sensor.                                             |
| noise_model_config            | JSON    | The configuration for the noise model.                                              |
| detection_cycle_delay_seconds | int     | The delay between sensor detections.                                                |
| infrastructure_id             | int     | The ID of the infrastructure.                                                       |
| sensor_id                     | int     | The ID of the sensor.                                                               |
| sensor_position               | float[] | Sensor position in CARLA world coordinates.                                         |
| sensor_rotation               | float[] | Sensor rotation in degrees.                                                         |
| parent_actor_id               | int     | ID of the parent actor to which the sensor is attached. Use -1 to leave unattached. |

#### Return

The registered SemanticLidarSensor object.

## get_simulated_sensor

### Input Parameters

**_infrastructure_id_** The ID of the infrastructure.

**_sensor_id_** The ID of the sensor.

#### Return

The registered sensor object.



### get_detected_objects

Retrieves the detected objects from a sensor.


#### Input Parameters

| Parameter                     | Type    | Description                                                                         |
|-------------------------------|---------|-------------------------------------------------------------------------------------|
| infrastructure_id             | int     | The ID of the infrastructure.                                                       |
| sensor_id                     | int     | The ID of the sensor.                                                               |

#### Return

The list of DetectedObjects detected by the sensor.


