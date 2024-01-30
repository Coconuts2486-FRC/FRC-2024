package frc.robot.Vision;
//raspberrypi

//Tagx
//Tagz
//TagId
//Ringz
//Ringx

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RaspberryPi {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("raspberrypi");
    public static PIDController PIDCO = new PIDController(0.1, 0, 0);

    public static double getTagX() {
        return table.getEntry("Tagx").getDouble(0.0);
    }

    public static double getTagZ() {
        return table.getEntry("Tagz").getDouble(0.0);
    }

    // Number is temporary
    public static Number getTagId() {
        return table.getEntry("TagId").getNumber(0);
    }

    public static double getRingz() {
        return table.getEntry("Ringz").getDouble(0.0);
    }

    public static double getRingx() {
        return table.getEntry("Ringx").getDouble(0.0);
    }

    public static double getTarget(boolean button, double axis) {
        if (button == true){
        return PIDCO.calculate(getTagX());
        }
        else{
            return axis;
        }
    }
}
