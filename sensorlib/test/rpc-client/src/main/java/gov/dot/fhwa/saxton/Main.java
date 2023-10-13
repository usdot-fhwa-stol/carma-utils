package gov.dot.fhwa.saxton;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.Arrays;
import java.util.List;

public class Main {

    static XmlRpcClient client;
    static Object[] params;

    public static void main(String[] args) throws XmlRpcException, MalformedURLException {

        // Connect
        XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
        config.setServerURL(new URL("http://127.0.0.1:8000/RPC2"));
        client = new XmlRpcClient();
        client.setConfig(config);

        // Specify sensor parameters
        int infrastructureID = 0;
        int sensorID = 7;
        List<Double> location = Arrays.asList(1.0, 2.0, 3.0);
        List<Double> rotation = Arrays.asList(0.0, 0.0, 0.0);
        params = new Object[]{"../config/simulated_sensor_config.yaml", "../config/noise_model_config.yaml", 0.5, infrastructureID, sensorID, location, rotation, -1};

        System.out.println();

        // Create sensor
        execute("create_simulated_semantic_lidar_sensor", params);

        // Retrieve sensor
        execute("get_simulated_sensor", new Object[]{infrastructureID, sensorID});

        // Get detected objects
        execute("get_detected_objects", new Object[]{infrastructureID, sensorID});

    }

    static void execute(String functionName, Object[] p) throws XmlRpcException {
        System.out.println("Executing " + functionName + "():");
        String result = (String) client.execute(functionName, p);
        System.out.println(result);
    }
}
