package frc.robot.Vision;
//raspberrypi

//TagX
//TagZ
//TagId
//RingZ
//RingX

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RaspberryPi {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("raspberrypi");
    public static PIDController PIDCO = new PIDController(0.1, 0, 0);
//function
    public static double getTagX() {
//what info we want from function
//This one gives us the X axis
        return table.getEntry("TagX").getDouble(0.0);
    }

    public static double getTagZ() {
//Z axis
        return table.getEntry("TagZ").getDouble(0.0);
    }

//!Number is temporary
//Gives us tag number so we can filter our tags we don't want
    public static Number getTagId() {
        return table.getEntry("TagId").getNumber(0);
    }
//Gives us Z axis for the game piece
    public static double getRingz() {
        return table.getEntry("RingZ").getDouble(0.0);
    }
//X axis for game piece
    public static double getRingx() {
        return table.getEntry("RingX").getDouble(0.0);
    }
//??
    public static double getTarget(boolean button, double axis) {
        if (button == true){
        return PIDCO.calculate(getTagX());
        }
        else{
            return axis;
        }
    }
}
