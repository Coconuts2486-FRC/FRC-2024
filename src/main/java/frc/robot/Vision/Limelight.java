package frc.robot.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public static double getY() {
        // Returns the Y axis of the target
        //return Math.round((table.getEntry("ty").getDouble(0.0) / 27f) * 100f) / 100f; // translated degrees -27 to 27 to a value between -1 and 1
        return table.getEntry("ty").getDouble(0.0);
        
    }
public static double testTagDistance() {

        double limelightMountAngleDegrees = 19.55/* aledgedly */;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 5.625;

        // distance from the target to the floor
        double goalHeightInches = 19.75;

        double angleToGoalDegrees = limelightMountAngleDegrees + getY();
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;

    }
}






