// NEEDS DOCUMENTING - 1/2/2024

package frc.robot;


import org.json.JSONException;
import org.json.JSONObject;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Simulation extends SubsystemBase {

    //Singleton
    public static Simulation Instance;
    private static int instanceNum = 0;

    public Simulation() {
        initialize();

        instanceNum++;
        if(instanceNum > 1) {
            
            System.out.println("ERROR: Simulation is a singletoon!  It can only be declared once.");
        }
        Instance = this;
    }


    public static String simulationCompatable(String realValue, String simulationKey, String simulationDefaultValue) {
        // Return realValue if the robot is actually running, and the simulation value if not
        if (RobotBase.isReal()) {
            return realValue;
        } else {
            return Simulation.Instance.getData(simulationKey, simulationDefaultValue);
        }
    }

    public static double simulationCompatable(double realValue, String simulationKey, double simulationDefaultValue) {
        if (RobotBase.isReal()) {
            return realValue;
        } else {
            return Simulation.Instance.getData(simulationKey, simulationDefaultValue);
        }
}

    private JSONObject transmitJsonData = new JSONObject("{}");

    private JSONObject receiveJsonData = new JSONObject("{}");

    private NetworkTableEntry transmitEntry;
    private NetworkTableEntry receiveEntry;
    
    public <T> void setData(String name, T value) {
        transmitJsonData.put(name, value);
    }

    public double getData(String name, double defaultValue) {
        try {
            return Double.parseDouble(receiveJsonData.get(name).toString());
        } catch(JSONException e) {
            return defaultValue;
        }
        
    }

    public String getData(String name, String defaultValue) {
        try {
            return receiveJsonData.get(name).toString();
        } catch(JSONException e) {
            return defaultValue;
        }
        
    }

    private void initialize() {
        transmitEntry = NetworkTableInstance.getDefault().getTable("Simulation").getEntry("RobotJSONData");
        receiveEntry = NetworkTableInstance.getDefault().getTable("Simulation").getEntry("SimulatorJSONData");
        transmitEntry.setString("{}");
        receiveEntry.setString("{}");
    }

    private void receive() {
        // Receive
        String receiveDataStr = receiveEntry.getString("{}");
        if (receiveDataStr.equals("{}")) return;
        receiveJsonData = new JSONObject(receiveDataStr);
    }

    private void transmit() {
        // Transmit
        transmitEntry.setString(transmitJsonData.toString());
    }

    @Override
    public void periodic() {
        receive();
        
        transmit();
    }

}
