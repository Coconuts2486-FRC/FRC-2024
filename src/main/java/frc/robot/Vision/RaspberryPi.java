package frc.robot.Vision;
//raspberrypi

//Tagx
//Tagz
//TagId
//Ringz
//Ringx

//blue is 7, red is 4

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RaspberryPi {

    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("vision");
    public static PIDController targetPid = new PIDController(0.1, 0, 0);
    public static PIDController driveToPid = new PIDController(1, 0.001, 0);

    // For April Tag 4
    public static double getTagX4() {
        return table.getEntry("speaker_tag4_robot_x").getDouble(0.0);
    }

    public static double getTagZ4() {
        return table.getEntry("speaker_tag4_robot_z").getDouble(0.0);
    }

      public static double getTagZ5() {
        return table.getEntry("speaker_tag5_robot_z").getDouble(0.0);
    }

    public static double getTagX5() {
        return table.getEntry("speaker_tag5_robot_x").getDouble(0.0);
    }

    public static double getTagZ6 (){
        return table.getEntry("speaker_tag6_robot_z").getDouble(0.0);
    }

    public static double getTagX6() {
        return table.getEntry("speaker_tag6_robot_x").getDouble(0.0);
    }


    // For April Tag 7
    public static double getTagX7() {
        return table.getEntry("speaker_tag7_robot_x").getDouble(0.0);
    }

    public static double getTagZ7() {
        return table.getEntry("speaker_tag7_robot_z").getDouble(0.0);
    }

    // Game piece stuff
    public static double gamePieceX() {
        return table.getEntry("gamepiece_robot_x").getDouble(0.0);
    }

    public static double gamePieceZ() {
        return table.getEntry("gamepiece_robot_z").getDouble(0.0);
    }

    public static double gamePieceAngle() {
        return table.getEntry("gamepiece_robot_angle").getDouble(0.0);
    }

    public static double gamePieceDistance() {
        return table.getEntry("gamepiece_robot_dist").getDouble(0.0);
    }

    public static double targetAprilTag(boolean button, double axis, boolean red) {
        if (red) {
            if (button) {
                return targetPid.calculate(getTagX4());
            } else {
                return axis;
            }
        } else {
            if (button) {
                return targetPid.calculate(getTagX7());
            } else {
                return axis;
            }
        }
    }

    public static double targetGamePiece() {

        return targetPid.calculate(gamePieceX());
    }

    public static double driveToGamePiece() {

        return driveToPid.calculate(gamePieceZ());

    }
}
