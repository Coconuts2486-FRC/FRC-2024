package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The RaspberryPi class provides methods to interact with vision data from a
 * Raspberry Pi.
 * //raspberrypi
 * Tagx
 * Tagz
 * TagId
 * Ringz
 * Ringx
 * blue is 7, red is 4
 */
public class RaspberryPi {
    public static boolean targetGamePieceToggle = false;
    public static double saveYaw;

    public static void init() {
        targetGamePieceToggle = false;
    }

    // The network table used for vision data.
    public static NetworkTable table = NetworkTableInstance
            .getDefault()
            .getTable("vision");

    // The PID controller used for targeting a specific value.
    public static PIDController targetPid = new PIDController(.58, 0, 0.0002);
    // public static PIDController targetPid = new PIDController(0.01, 0, 0.000002);
    public static PIDController targetPid2 = new PIDController(0.0095, 0, 0.000002);

    // The PID controller used for driving to a specific value.
    public static PIDController driveToPid = new PIDController(.03, 0.00, 0);

    /**
     * Retrieves the X coordinate of SPEAKER CENTER AprilTag, relative to the robot
     * 
     * @param red Are we red alliance?
     * @return The X coordinate of SPEAKER CENTER AprilTag
     */
    public static double getSpeakerCenterX(boolean red) {
        if (red) {
            return getTagX4();
        } else {
            return getTagX7();
        }
    }

    /**
     * Retrieves the Z coordinate of SPEAKER CENTER AprilTag, relative to the robot
     * 
     * @param red Are we red alliance?
     * @return The Z coordinate of SPEAKER CENTER AprilTag
     */
    public static double getSpeakerCenterZ(boolean red) {
        if (red) {
            return getTagZ4();
        } else {
            return getTagZ7();
        }
    }

    /**
     * Retrieves the X coordinate of April Tag 4 relative to the robot.
     *
     * @return The X coordinate of April Tag 4.
     */
    public static double getTagX4() {
        return table.getEntry("speaker_tag4_robot_x").getDouble(0.0);
    }

    /**
     * Retrieves the Z coordinate of April Tag 4 relative to the robot.
     *
     * @return The Z coordinate of April Tag 4.
     */
    public static double getTagZ4() {
        return table.getEntry("speaker_tag4_robot_z").getDouble(-999);
    }

    /**
     * Retrieves the Z coordinate of April Tag 5 relative to the robot.
     *
     * @return The Z coordinate of April Tag 5.
     */
    public static double getTagZ5() {
        return table.getEntry("speaker_tag5_robot_z").getDouble(0.0);
    }

    /**
     * Retrieves the X coordinate of April Tag 5 relative to the robot.
     *
     * @return The X coordinate of April Tag 5.
     */
    public static double getTagX5() {
        return table.getEntry("speaker_tag5_robot_x").getDouble(0.0);
    }

    /**
     * Retrieves the Z coordinate of April Tag 6 relative to the robot.
     *
     * @return The Z coordinate of April Tag 6.
     */
    public static double getTagZ6() {
        return table.getEntry("speaker_tag6_robot_z").getDouble(0.0);
    }

    /**
     * Retrieves the X coordinate of April Tag 6 relative to the robot.
     *
     * @return The X coordinate of April Tag 6.
     */
    public static double getTagX6() {
        return table.getEntry("speaker_tag6_robot_x").getDouble(0.0);
    }

    /**
     * Retrieves the X coordinate of April Tag 7 relative to the robot.
     *
     * @return The X coordinate of April Tag 7.
     */
    public static double getTagX7() {
        return table.getEntry("speaker_tag7_robot_x").getDouble(0.0);
    }

    /**
     * Retrieves the Z coordinate of April Tag 7 relative to the robot.
     *
     * @return The Z coordinate of April Tag 7.
     */
    public static double getTagZ7() {
        return table.getEntry("speaker_tag7_robot_z").getDouble(-999);
    }

    /**
     * Retrieves the X coordinate of the game piece relative to the robot.
     *
     * @return The X coordinate of the game piece.
     */
    public static double gamePieceX() {

        if (table.getEntry("gamepiece_robot_x").getDouble(0.0) == -999) {
            return 0;
        } else {
            return table.getEntry("gamepiece_robot_x").getDouble(0.0);
        }
    }

    /**
     * Retrieves the Z coordinate of the game piece relative to the robot.
     *
     * @return The Z coordinate of the game piece.
     */
    public static double gamePieceZ() {

        if (table.getEntry("gamepiece_robot_z").getDouble(0.0) == -999) {
            return 0;
        } else {
            return table.getEntry("gamepiece_robot_z").getDouble(0.0);
        }
    }

    /**
     * Retrieves the angle of the game piece relative to the robot.
     *
     * @return The angle of the game piece.
     */
    public static double gamePieceAngle() {
        if (table.getEntry("gamepiece_robot_angle").getDouble(0.0) == -999) {
            return 0;
        } else {
            return table.getEntry("gamepiece_robot_angle").getDouble(0.0);
        }

    }

    /**
     * Retrieves the distance to the game piece relative to the robot.
     *
     * @return The distance to the game piece.
     */
    public static double gamePieceDistance() {
        return table.getEntry("gamepiece_robot_dist").getDouble(0.0);
    }

    /**
     * Calculates the target value for April Tags based on the button and axis
     * inputs.
     *
     * @param button The button input.
     * @param axis   The axis input.
     * @param red    Are we red alliance?
     * @return The target value for April Tags.
     */
    public static double targetAprilTag(
            boolean button,
            double axis,
            boolean red) {
        double tagXPosition = getSpeakerCenterX(red);
        if (button) {
            if (tagXPosition == -999) {
                return axis;
            } else {
               return Math.signum(tagXPosition)*.083;
               // return -targetPid2.calculate(());
            }
        } else {
            return axis;
        }

    }

    /**
     * Calculates the target value for the game piece.
     *
     * @param button   This is the magical "Button 6" pressed
     * @param released Not used
     * @return The target value for the game piece.
     */
    public static void targetGamePiece(boolean button, boolean released) {
        // If button, no gamepiece in intake, and intake is OUT
        if (button && Map.lightStop.get() == false && Map.intakeExtend.getSelectedSensorPosition() > 90000) {
            Map.swerve.drive(0, 0, -targetPid.calculate(gamePieceAngle()), false);
            if (Math.abs(gamePieceX()) < 7) {
                // Swerve.gyro.setYaw(0); // Instead, call getRobotAngle()
                Map.swerve.drive(0, .3, -targetPid.calculate(gamePieceAngle()), true);
            }

        }

        // Map.backLeft.autoInit(Swerve.blOffset);
        // Map.backRight.autoInit(Swerve.brOffset);
        // Map.frontLeft.autoInit(Swerve.flOffset);
        // Map.frontRight.autoInit(Swerve.frOffset);
        // Swerve.modInit();

    }

    /**
     * Calculates the target value for the gamepiece
     * 
     * Secondary function DOES NOT reset the `gyro` yaw -- possibly will allow
     * us to remove second gyro on the bot (3-6-24)
     * 
     * TESTING!!!!
     * If possible, assign an unallocated controller button to this function for
     * testing
     * 
     * @param button
     */
    public static void targetGamepiece2(boolean button) {
        // If button, no gamepiece in intake, and intake is OUT
        if (button && Map.lightStop.get() == false && Map.intakeExtend.getSelectedSensorPosition() > 90000) {
            Map.swerve.drive(0, 0, -targetPid.calculate(gamePieceX()), false);
            if (Math.abs(gamePieceX()) < 7) {

                // This is the current robot rotation angle w.r.t. initialization
                double theta = Map.swerve.getRobotAngle(); // radians (might need a minus sign)

                // This is where we want to go w.r.t. our current orientation
                double xp = 0; // Want the robot to NOT move in x'
                double yp = -driveToPid.calculate(gamePieceZ()); // Move the robot toward the gamepiece

                // Use the 2D rotation matrix to compute the swerve drive motion
                // based on current YAW and desired robot-centric motion
                Map.swerve.drive(xp * Math.cos(theta - yp * Math.sin(theta)),
                        xp * Math.sin(theta) + yp * Math.cos(theta), -targetPid.calculate(gamePieceX()), false);
            }

        }
    }

    /**
     * Calculates the drive-to value for the game piece.
     *
     * @return The drive-to value for the game piece.
     */
    public static double driveToGamePiece() {
        if (Map.lightStop.get()) {
            return 0;
        } else {
            return -driveToPid.calculate(gamePieceZ());
        }
    }
}
