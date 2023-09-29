# CARLA Sensor Library Service

The CARLA Sensor Library Service provides an XML-RPC service exposing sensor creation and management capability, and interfacing with the simulated sensor algorithm to retrieve objects which are detected in the scene.

## Launching

The service is started by running the Python main function as follows:

```
python3 SensorDataService.py --carla-host <CARLA_HOST> --carla_port <CARLA_PORT> --port <PORT>




usage: SensorDataService.py [-h] [--infrastructure-id INFRASTRUCTURE_ID] [--sensor-id SENSOR_ID] [--carla-host CARLA_HOST] [--carla-port CARLA_PORT] [--xmlrpc-server-host XMLRPC_SERVER_HOST] [--xmlrpc-server-port XMLRPC_SERVER_PORT]

optional arguments:
  -h, --help            show this help message and exit
  --infrastructure-id INFRASTRUCTURE_ID
                        Infrastructure ID to associate the sensor to.
  --sensor-id SENSOR_ID
                        Sensor ID to assign.
  --carla-host CARLA_HOST
                        CARLA host. (default: "127.0.0.1")
  --carla-port CARLA_PORT
                        CARLA host. (default: "2000")
  --xmlrpc-server-host XMLRPC_SERVER_HOST
                        XML-RPC server host. (default: "localhost")
  --xmlrpc-server-port XMLRPC_SERVER_PORT
                        XML-RPC server port. (default: 8000)








```

### create_simulated_semantic_lidar_sensor

Builds a SemanticLidarSensor from a CARLA Semantic LIDAR Sensor.

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

Integer sensor ID for the registered SemanticLidarSensor, confirming successful registration.


### get_detected_objects

Retrieves the detected objects from a sensor.

#### Input Parameters

| Parameter                     | Type    | Description                                                                         |
|-------------------------------|---------|-------------------------------------------------------------------------------------|
| infrastructure_id             | int     | The ID of the infrastructure.                                                       |
| sensor_id                     | int     | The ID of the sensor.                                                               |

#### Return

JSON-serialized string containing the list of detected objects.
