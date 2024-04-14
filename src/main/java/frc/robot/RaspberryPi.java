package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

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
    public static int AmpIndex = 0;
    public static double delay = 0;
    public static double hold;
       static double  ampTagZ;

    public static void init() {
        targetGamePieceToggle = false;
        AmpIndex = 0;
        
    }

    // The network table used for vision data.
    public static NetworkTable table = NetworkTableInstance
            .getDefault()
            .getTable("vision");

    // The PID controller used for targeting a specific value.
    public static PIDController twistPidFar = new PIDController(0.007, 0.00, 0.00000);
        public static PIDController twistPidClose = new PIDController(0.04, 0.00, 0.00000);
    public static PIDController noteTargetPidClose = new PIDController(.55, 0, 0.00014);
    public static PIDController noteTargetPidFar = new PIDController(.3, 0, 0.00007);

    // public static PIDController targetPid = new PIDController(0.01, 0, 0.000002);
    public static PIDController tagTargetPidClose = new PIDController(.8, 0.00, 0.00005);

    public static PIDController tagTargetPidFar = new PIDController(0.5, 0.00, 0.00001);

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

    public static double getSpeakerCenterA(boolean red) {
        if (red) {
            return getTagA4();
        } else {
            return getTagA7();
        }
    }

    public static double getSpeakerCenterZ(boolean red) {
        if (red) {
            return getTagZ4();
        } else {
            return getTagZ7();
        }
    }

    public static double getAmpCenterX(boolean red) {
        if (red) {
            return getTagX5();
        } else {
            return getTagX6();
        }
    }

    public static double getAmpCenterA(boolean red) {
        if (red) {
            return getTagA5();
        } else {
            return getTagA6();
        }
    }

    public static double getAmpCenterZ(boolean red) {
        if (red) {
            return getTagZ5();
        } else {
            return getTagZ6();
        }
    }

    /**
     * Retrieves the Z coordinate of SPEAKER CENTER AprilTag, relative to the robot
     * 
     * @param red Are we red alliance?
     * @return The Z coordinate of SPEAKER CENTER AprilTag
     */
    public static double getSpeakerSideZ(boolean red) {
        if (red) {
            return getTagZ3();
        } else {
            return getTagZ8();
        }
    }

    public static double getSpeakerSideX(boolean red) {
        if (red) {
            return getTagX3();
        } else {
            return getTagX8();
        }
    }

    public static double getSpeakerSideA(boolean red) {
        if (red) {
            return getTagA3();
        } else {
            return getTagA8();
        }
    }

    /**
     * Retrieves the Z coordinate of SPEAKER CENTER AprilTag, relative to the robot
     * 
     * @param red Are we red alliance?
     * @return The Z coordinate of SPEAKER CENTER AprilTag
     */

    /**
     * Retrieves the X coordinate of April Tag 3 relative to the robot.
     *
     * @return The X coordinate of April Tag 3.
     */
    public static double getTagX3() {
        return table.getEntry("speaker_tag3_robot_x").getDouble(0.0);
    }

    public static double getTagA3() {
        return table.getEntry("speaker_tag3_robot_ang").getDouble(0.0);
    }

    /**
     * Retrieves the Z coordinate of April Tag 3 relative to the robot.
     *
     * @return The Z coordinate of April Tag 3.
     */
    public static double getTagZ3() {
        return table.getEntry("speaker_tag3_robot_z").getDouble(-999);
    }

    /**
     * Retrieves the X coordinate of April Tag 4 relative to the robot.
     *
     * @return The X coordinate of April Tag 4.
     */
    public static double getTagX4() {
        return table.getEntry("speaker_tag4_robot_x").getDouble(0.0);
    }

    public static double getTagA4() {
        return table.getEntry("speaker_tag4_robot_ang").getDouble(0.0);
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
        return table.getEntry("amp_tag5_robot_z").getDouble(0.0);
    }

    /**
     * Retrieves the X coordinate of April Tag 5 relative to the robot.
     *
     * @return The X coordinate of April Tag 5.
     */
    public static double getTagX5() {
        return table.getEntry("amp_tag5_robot_x").getDouble(0.0);
    }

    public static double getTagA5() {
        return table.getEntry("amp_tag5_robot_ang").getDouble(0.0);
    }

    /**
     * Retrieves the Z coordinate of April Tag 6 relative to the robot.
     *
     * @return The Z coordinate of April Tag 6.
     */
    public static double getTagZ6() {
        return table.getEntry("amp_tag6_robot_z").getDouble(0.0);
    }

    /**
     * Retrieves the X coordinate of April Tag 6 relative to the robot.
     *
     * @return The X coordinate of April Tag 6.
     */
    public static double getTagX6() {
        return table.getEntry("amp_tag6_robot_x").getDouble(0.0);
    }

    public static double getTagA6() {
        return table.getEntry("amp_tag6_robot_ang").getDouble(0.0);
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

    public static double getTagA7() {
        return table.getEntry("speaker_tag7_robot_ang").getDouble(0.0);
    }

    /**
     * Retrieves the X coordinate of the game piece relative to the robot.
     *
     * @return The X coordinate of the game piece.
     */

    /**
     * Retrieves the X coordinate of April Tag 8 relative to the robot.
     *
     * @return The X coordinate of April Tag 8.
     */
    public static double getTagX8() {
        return table.getEntry("speaker_tag8_robot_x").getDouble(0.0);
    }

    public static double getTagA8() {
        return table.getEntry("speaker_tag8_robot_ang").getDouble(0.0);
    }

    /**
     * Retrieves the Z coordinate of April Tag 8 relative to the robot.
     *
     * @return The Z coordinate of April Tag 8.
     */
    public static double getTagZ8() {
        return table.getEntry("speaker_tag8_robot_z").getDouble(-999);
    }

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
        double tagAPosition = getSpeakerCenterA(red);
        double tagXPosition = getSpeakerCenterX(red);
        double tagZPosition = getSpeakerCenterZ(red);
        double velocity;
        // if (Math.abs(tagXPosition) > 27) {
        // velocity = .22 * Math.signum(tagXPosition);
        // } else if (Math.abs(tagXPosition) > 21) {
        // velocity = .16 * Math.signum(tagXPosition);
        // }else if (){

        // } else if (Math.abs(tagXPosition) >= 12) {
        // velocity = .12 * Math.signum(tagXPosition);
        // } else if (Math.abs(tagXPosition) >= 6) {
        // velocity = .09 * Math.signum(tagXPosition);
        // } else
        if (Math.abs(tagZPosition) > 100) {
            if (Math.abs(tagXPosition) > 14) {
                velocity = -tagTargetPidFar.calculate(tagAPosition);
            } else {
                velocity = -tagTargetPidClose.calculate(tagAPosition);
            }
        } else {
            if (Math.abs(tagXPosition) > 12) {
                velocity = -tagTargetPidFar.calculate(tagAPosition);
            } else {
                velocity = -tagTargetPidClose.calculate(tagAPosition);
            }
        }
        if (button) {
            if (tagAPosition == -999) {
                return axis;
            } else {
                // return Math.signum(tagXPosition) * .083;
                return velocity;
                // return -targetPid2.calculate(tagXPosition);
            }
        } else {
            return axis;
        }

    }

    public static double targetAprilTagSide(
            boolean button,
            double axis,
            boolean red) {
        double tagAPosition = getSpeakerSideA(red);
        double tagXPosition = getSpeakerSideX(red);
        double tagZPosition = getSpeakerSideZ(red);
        double velocity;
        // if (Math.abs(tagXPosition) > 27) {
        // velocity = .22 * Math.signum(tagXPosition);
        // } else if (Math.abs(tagXPosition) > 21) {
        // velocity = .16 * Math.signum(tagXPosition);
        // }else if (){

        // } else if (Math.abs(tagXPosition) >= 12) {
        // velocity = .12 * Math.signum(tagXPosition);
        // } else if (Math.abs(tagXPosition) >= 6) {
        // velocity = .09 * Math.signum(tagXPosition);
        // } else
        if (Math.abs(tagZPosition) > 100) {
            if (Math.abs(tagXPosition) > 14) {
                velocity = -tagTargetPidFar.calculate(tagAPosition);
            } else {
                velocity = -tagTargetPidClose.calculate(tagAPosition);
            }
        } else {
            if (Math.abs(tagXPosition) > 12) {
                velocity = -tagTargetPidFar.calculate(tagAPosition);
            } else {
                velocity = -tagTargetPidClose.calculate(tagAPosition);
            }
        }
        if (button) {
            if (tagAPosition == -999) {
                return axis;
            } else {
                // return Math.signum(tagXPosition) * .083;
                return velocity;
                // return -targetPid2.calculate(tagXPosition);
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
    public static void targetGamePiece(boolean button, boolean gently) {
        // If button, no gamepiece in intake, and intake is OUT
        double velocity;
        double rotation;
        if (Math.abs(gamePieceX()) < 14) {
            rotation = -noteTargetPidClose.calculate(gamePieceAngle());
        } else {
            rotation = -noteTargetPidFar.calculate(gamePieceAngle());
        }
        if (button && Map.lightStop.get() == false && Map.intakeExtend.getSelectedSensorPosition() > 89000) {
            Map.swerve.drive(0, 0, rotation, false);
            if (Math.abs(gamePieceX()) < 7) {
                // Taper the velocity to zero in the last foot
                if (gently) {
                    velocity = Math.max(gamePieceZ() / 36., 1) * 0.25;
                } else {
                    velocity = Math.max(gamePieceZ() / 36., 1) * 0.45;
                }
                // Swerve.gyro.setYaw(0); // Instead, call getRobotAngle()
                Map.swerve.drive(0, velocity, rotation, true);
            }

        }

        // Map.backLeft.autoInit(Swerve.blOffset);
        // Map.backRight.autoInit(Swerve.brOffset);
        // Map.frontLeft.autoInit(Swerve.flOffset);
        // Map.frontRight.autoInit(Swerve.frOffset);
        // Swerve.modInit();

    }

    public static void targetGamePieceAuto(boolean button, boolean released) {
        // If button, no gamepiece in intake, and intake is OUT
        if (button && Map.lightStop.get() == false && Map.intakeExtend.getSelectedSensorPosition() > 89000) {
            Map.swerve.drive(0, 0, -noteTargetPidFar.calculate(gamePieceAngle()), false);
            if (Math.abs(gamePieceX()) < 7) {
                // Swerve.gyro.setYaw(0); // Instead, call getRobotAngle()
                Map.swerve.drive(0, .45, -noteTargetPidFar.calculate(gamePieceAngle()), true);
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
            Map.swerve.drive(0, 0, -noteTargetPidFar.calculate(gamePieceX()), false);
            if (Math.abs(gamePieceX()) < 7) {

                // This is the current robot rotation angle w.r.t. initialization
                double theta = Map.swerve.getRobotAngle(); // radians (might need a minus sign)

                // This is where we want to go w.r.t. our current orientation
                double xp = 0; // Want the robot to NOT move in x'
                double yp = -driveToPid.calculate(gamePieceZ()); // Move the robot toward the gamepiece

                // Use the 2D rotation matrix to compute the swerve drive motion
                // based on current YAW and desired robot-centric motion
                Map.swerve.drive(xp * Math.cos(theta - yp * Math.sin(theta)),
                        xp * Math.sin(theta) + yp * Math.cos(theta), -noteTargetPidFar.calculate(gamePieceX()), false);
            }

        }
    }

    public static double twist(double currentAngle, double setPoint){
        if(Math.abs(currentAngle-setPoint)>5){
            return Math.signum(-currentAngle+setPoint) * 0.2 * Math.max(1, Math.abs(currentAngle - setPoint)/30);
        //     return twistPidFar.calculate(currentAngle,setPoint);

        }else{
           return twistPidClose.calculate(currentAngle,setPoint);

        }
    }

    public static void targetAmp(boolean button, boolean red, boolean buttonRelease) {
      double  ampTagX;
   
      if (getAmpCenterX(red) == -999){
            ampTagX = 0;
      }
       else {
         ampTagX = RaspberryPi.getAmpCenterX(red);
       }

        if (button) {
            if (AmpIndex == 0) {
                if (red) {
                    Map.swerve.drive(0, 0, twist(Swerve.gyro.getYaw(), 90), false);
                      if(Math.abs(90-Swerve.gyro.getYaw())<1){
                        delay = Timer.getFPGATimestamp();
                        AmpIndex = 1;
                      }
                } else {
                    Map.swerve.drive(0, 0, twist(Swerve.gyro.getYaw(), 270), false);
                    if(Math.abs(270-Swerve.gyro.getYaw())<1){
                        delay = Timer.getFPGATimestamp();
                        AmpIndex = 1;
                    }
                }
                
            } else if (AmpIndex == 1){
                 Map.swerve.drive(0, 0,0, false);
                 if ( Timer.getFPGATimestamp() - delay > .0){
                    AmpIndex = 2;
                 }
            } 
            
            
            if (AmpIndex == 2) {
                if (red) {
                    Map.swerve.drive(0, 0, twist(Swerve.gyro.getYaw(), 90), false);
                      if(Math.abs(90-Swerve.gyro.getYaw())<1){
                        delay = Timer.getFPGATimestamp();
                        AmpIndex = 3;
                      }
                } else {
                    Map.swerve.drive(0, 0, twist(Swerve.gyro.getYaw(), 270), false);
                    if(Math.abs(270-Swerve.gyro.getYaw())<1){
                        delay = Timer.getFPGATimestamp();
                        AmpIndex = 3;
                    }
                }
                
            } else if (AmpIndex == 3){
                 Map.swerve.drive(0, 0,0, false);
                 if ( Timer.getFPGATimestamp() - delay > .0){
                    AmpIndex = 4;
                 }
            }
            if (AmpIndex == 4) {
                if (red) {
                    Map.swerve.drive(0, 0, twist(Swerve.gyro.getYaw(), 90), false);
                      if(Math.abs(90-Swerve.gyro.getYaw())<2.5){
                        delay = Timer.getFPGATimestamp();
                        AmpIndex = 5;
                      }
                } else {
                    Map.swerve.drive(0, 0, twist(Swerve.gyro.getYaw(), 270), false);
                    if(Math.abs(270-Swerve.gyro.getYaw())<2.5){
                        delay = Timer.getFPGATimestamp();
                        AmpIndex = 5;
                    }
                }
                
            } else if (AmpIndex == 5){
                 Map.swerve.drive(0, 0,0, false);
                 if ( Timer.getFPGATimestamp() - delay > .2){
                    hold = Swerve.gyro.getYaw();
                    AmpIndex = 6;

                 }
            }
            else if (AmpIndex == 6){

                Map.swerve.drive(driveToPid.calculate(ampTagX), 0, Auto.twistPid.calculate(Swerve.gyro.getYaw(),hold), true);
                if (Math.abs(RaspberryPi.getAmpCenterX(red))<5){
                    AmpIndex = 8;
                } 
            } else if (AmpIndex == 7){
                 Map.swerve.drive(0, 0, 0, true);
                 Map.intakeBottom.set(ControlMode.PercentOutput,.25);
                   Map.intakeTop.set(ControlMode.PercentOutput,.25);
                   if(Map.lightStop.get()==false){
                    AmpIndex = 8;
                   }

            }else if (AmpIndex == 8){
                 Map.swerve.drive(0, 0, 0, true);
                 Map.intakeBottom.set(ControlMode.PercentOutput,.25);
                   Map.intakeTop.set(ControlMode.PercentOutput,.25);
                   if(Map.lightStop.get()){
                    Map.intakeBottom.set(ControlMode.PercentOutput,0);
                   Map.intakeTop.set(ControlMode.PercentOutput,0);
                    ampTagZ = getAmpCenterZ(red);
                    Map.frontRight.driveMotor.setSelectedSensorPosition(0);

                    hold = Swerve.gyro.getYaw();
                    AmpIndex = 9;
                   }
                } else if ( AmpIndex == 9){
                    
                     Map.swerve.drive(0, .5,0,true);// Auto.holdPid.calculate(hold), true);
                     if (Math.abs(Map.frontRight.driveMotor.getSelectedSensorPosition())>(ampTagZ*2120)){
                        AmpIndex = 10;
                     }

                }else if( AmpIndex == 10){
                      Map.swerve.drive(0, .2, 0, true);
                }
        } else {
            AmpIndex = 0;
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
