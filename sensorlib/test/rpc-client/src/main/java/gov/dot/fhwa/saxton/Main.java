package gov.dot.fhwa.saxton;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.Arrays;
import java.util.List;
import java.lang.Thread;

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
        int infrastructureID = 1;
        int sensorID = 7;
        List<Double> location = Arrays.asList(63.0, 7.0, 1.8);
        List<Double> rotation = Arrays.asList(0.0, 0.0, 0.0);
        params = new Object[]{ infrastructureID, sensorID, location, rotation};

        // Create sensor
        execute("create_simulated_semantic_lidar_sensor", params);

        // Retrieve sensor
        execute("get_simulated_sensor", new Object[]{infrastructureID, sensorID});
        int timer = 0;
        // Get detected objects
        while (timer < 10)
        {
            execute("get_detected_objects", new Object[]{infrastructureID, sensorID});
            try {
                // Sleep for 1 sec
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            timer++;
        }

    }

    static void execute(String functionName, Object[] p) throws XmlRpcException {
        System.out.println("Executing " + functionName + "():");
        try{
            String result = (String) client.execute(functionName, p);
            System.out.println(result);

        }
        catch(XmlRpcException e) {
            e.printStackTrace(System.out);
        }
    }
}