# Carla CDA Sim Adapter

The Carla CDA Sim Adapter instantiates an XML-RPC server providing the sensorlib capabilities.

Prior to starting the service, set the following environment variables to define the CARLA version being utilized:

| Environment Variable | Description                                                                                                       |
| -------------------- |-------------------------------------------------------------------------------------------------------------------|
| LOAD_CARLA_EGG       | Set to True to load the CARLA Python API from an egg file. Leave blank to use a pip-managed CARLA client library. |
| CARLA_VERSION        | The CARLA version to load (ex: "0.0.14").                                                                         |
| CARLA_EGG_DIR        | (optional) Hint directory for searching for the CARLA egg file.                                                   |

To launch the service using the command line, run the `CarlaCDASimAdapter.py` file using the Python interpreter and enter
any desired arguments.

```
python3 CarlaCDASimAdapter.py  --carla-host 192.168.100.10
```

At this point, an XML-RPC service is started which provides remote access to register sensors and retrieve detection
data. This is demonstrated in this sample Java code.

```java
public static void main(String[] args) throws XmlRpcException, MalformedURLException {

    // Connect
    XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
    config.setServerURL(new URL("http://127.0.0.1:8000/RPC2"));
    XmlRpcClient client = new XmlRpcClient();
    client.setConfig(config);

    // Specify sensor parameters
    int infrastructureID = 0;
    int sensorID = 7;
    List<Double> location = Arrays.asList(1.0, 2.0, 3.0);
    List<Double> rotation = Arrays.asList(0.0, 0.0, 0.0);
    Object[] params = new Object[]{"config/simulated_sensor_config.yaml", "config/noise_model_config.yaml", 0.5, infrastructureID, sensorID, location, rotation, -1};

    // Create sensor
    String result;
    result = (String) client.execute("create_simulated_semantic_lidar_sensor", params);

    // Retrieve sensor
    result = (String) client.execute("get_simulated_sensor", new Object[]{infrastructureID, sensorID});

    // Get detected objects
    result = (String) client.execute("get_detected_objects", new Object[]{infrastructureID, sensorID});
}
```

These functions return string ID's and in the case of `get_detected_objects()`, a serialized JSON representation of the
underlying data.

# Service Interface

The service may be started at command line and by passing command-line arguments. Available arguments are:

```
usage: CarlaCDASimAdapter.py [-h] [--carla-host CARLA_HOST] [--carla-port CARLA_PORT]
                            [--xmlrpc-server-host XMLRPC_SERVER_HOST] [--xmlrpc-server-port XMLRPC_SERVER_PORT]

optional arguments:
  -h, --help            show this help message and exit
  --carla-host CARLA_HOST
                        CARLA host. (default: "127.0.0.1")
  --carla-port CARLA_PORT
                        CARLA host. (default: "2000")
  --xmlrpc-server-host XMLRPC_SERVER_HOST
                        XML-RPC server host. (default: "localhost")
  --xmlrpc-server-port XMLRPC_SERVER_PORT
                        XML-RPC server port. (default: 8000)
  --sensor-config-file
                        Configuration file used for sensor creation.(*default :"./config/simulated_sensor_config.yaml")
  --noise-model-config-file
                        Configuration file used for setting up noise model for sensors(*default :"./config/noise_model_config.yaml")
  --detection-cycle-delay-seconds
                        Time interval in seconds between detection reporting. (default: 0.5)
```

# XML-RPC Interface

The following are the exposed RPC functions:

     |  create_simulated_semantic_lidar_sensor(self, infrastructure_id, sensor_id, sensor_position, sensor_rotation)
     |      Builds a SemanticLidarSensor from a CARLA Semantic LIDAR Sensor.
     |      
     |      :param infrastructure_id: The ID of the infrastructure.
     |      :param sensor_id: The ID of the sensor.
     |      :param sensor_position: Sensor position in CARLA world coordinates.
     |      :param sensor_rotation: Sensor rotation in degrees.
     |      :return: ID of the registered SimulatedSensor.
     |  
     |  get_simulated_sensor(self, infrastructure_id, sensor_id)
     |      Get a specific simulated sensor.
     |      
     |      :param infrastructure_id: The ID of the infrastructure.
     |      :param sensor_id: The ID of the sensor.
     |      :return: The ID of the SimulatedSensor.
     |      
     |  get_detected_objects(self, infrastructure_id, sensor_id)
     |      Get the detected objects from a specific sensor. The DetectedObject's are those found by the sensor in the most
     |      recent call to SimulatedSensor.compute_detected_objects().
     |      
     |      :param infrastructure_id: The ID of the infrastructure.
     |      :param sensor_id: The ID of the sensor.
     |      :return: List of DetectedObject's discovered by the associated sensor, in serialized JSON form. Each entry contains metadata including object id, position and type.

# CarlaCDASimAdapter Class

## Description

Provides a programmatic interface to launch the XML-RPC service. To launch, build a CarlaCDASimAPI object and pass into the
constructor of a CarlaCDASimAdapter. Then call start_xml_rpc_server(), setting the `blocking` parameter to easily block
or start the server on a separate thread.

````
api = CarlaCDASimAPI.build_from_host_spec(carla_host, carla_port)
sensor_data_service = CarlaCDASimAdapter(api)
sensor_data_service.start_xml_rpc_server(args.xmlrpc_server_host, args.xmlrpc_server_port, False)
````

## Interface

    class CarlaCDASimAdapter
     |  CarlaCDASimAdapter(sensor_api)
     |  
     |  Methods defined here:
     |  
     |  __init__(self, sensor_api)
     |      CarlaCDASimAdapter constructor.
     |      :param sensor_api: The API object exposing CARLA connection.
     |  
     |  start_xml_rpc_server(self, xmlrpc_server_host, xmlrpc_server_port, blocking=True)
     |      Starts the XML-RPC server for the sensor data service.
     |      :param xmlrpc_server_host: The server host.
     |      :param xmlrpc_server_port: The server port.
     |      :param blocking: Toggles server starting on main thread. If False the server will be started on a new thread, and control will be returned to the caller.
     |      :return: True if the server was started on a new thread. Nothing is returned if the server was started on the main thread.

# File

    CarlaCDASimAdapter.py
