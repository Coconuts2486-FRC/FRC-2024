package frc.robot.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static PIDController limelightPID = new PIDController(.01,0,0);
    public static double getY() {
        // Returns the Y axis of the target
        return table.getEntry("ty").getDouble(0.0);
        
    }
    public static double getX() {
        // Returns the X axis of the target
        return table.getEntry("tx").getDouble(0.0);
        
    }
public static double testTagDistance() {

        double limelightMountAngleDegrees = 21/*aledgedly*/;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 5.625;

        // distance from the target to the floor
        double goalHeightInches = 58.375;

        double angleToGoalDegrees = limelightMountAngleDegrees + getY();
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;

    }
    public static double tagTarget(boolean button,double axis) {
        if (button){
            double targetVal = -limelightPID.calculate(getX());
            return targetVal;
        }else{
            return axis;
        }
    }
}






