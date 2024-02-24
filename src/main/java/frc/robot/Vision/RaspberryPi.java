package frc.robot.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Map;
import frc.robot.Swerve;

/**
 * The RaspberryPi class provides methods to interact with vision data from a Raspberry Pi.
 * //raspberrypi
 *Tagx
 *Tagz
 *TagId
 *Ringz
 *Ringx
 *blue is 7, red is 4
 */
public class RaspberryPi {
  public static boolean targetGamePieceToggle = false;
     public static double saveYaw;
     private static int zeroTrippleToggle;


  public static void init(){
     targetGamePieceToggle = false;
  }
  /**
   * The network table used for vision data.
   */
  public static NetworkTable table = NetworkTableInstance
    .getDefault()
    .getTable("vision");

  /**
   * The PID controller used for targeting a specific value.
   */
  public static PIDController targetPid = new PIDController(0.015, 0, 0.0000001);

  /**
   * The PID controller used for driving to a specific value.
   */
  public static PIDController driveToPid = new PIDController(.08, 0.001, 0);

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
    return table.getEntry("speaker_tag4_robot_z").getDouble(0.0);
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
    return table.getEntry("speaker_tag7_robot_z").getDouble(0.0);
  }

  /**
   * Retrieves the X coordinate of the game piece relative to the robot.
   *
   * @return The X coordinate of the game piece.
   */
  public static double gamePieceX() {
     
    if ( table.getEntry("gamepiece_robot_x").getDouble(0.0) == -999){
      return 0;
    }else{
       return table.getEntry("gamepiece_robot_x").getDouble(0.0);
    }
  }


  /**
   * Retrieves the Z coordinate of the game piece relative to the robot.
   *
   * @return The Z coordinate of the game piece.
   */
  public static double gamePieceY() {
   
    if ( table.getEntry("gamepiece_robot_z").getDouble(0.0) == -999){
      return 0;
    }else{
       return table.getEntry("gamepiece_robot_z").getDouble(0.0);
    }
  }

  /**
   * Retrieves the angle of the game piece relative to the robot.
   *
   * @return The angle of the game piece.
   */
  public static double gamePieceAngle() {
    return table.getEntry("gamepiece_robot_angle").getDouble(0.0);
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
   * Calculates the target value for April Tags based on the button and axis inputs.
   *
   * @param button The button input.
   * @param axis   The axis input.
   * @param red    Whether the target is red or not.
   * @return The target value for April Tags.
   */
  public static double targetAprilTag(
    boolean button,
    double axis,
    boolean red
  ) {
    if (red) {
      if (button) {
        if (getTagX4()==-999){
          return axis;
        }else{
        return -targetPid.calculate(getTagX4());
        }
      } else {
        return axis;
      }
    } else {
      if (button) {
        if (getTagX7() == -999){
          return axis;
        }else{
        return targetPid.calculate(getTagX7());
        }
      } else {
        return axis;
      }
    }
  }

  /**
   * Calculates the target value for the game piece.
   *
   * @return The target value for the game piece.
   */
  public static void targetGamePiece(boolean toggleButton, boolean button, boolean toggleRelease) {
    if (button && Map.lightStop.get()== false){
      Map.swerve.drive(0,0,-targetPid.calculate(gamePieceX()));
      if(Math.abs(gamePieceX())<7){
        Swerve.gyro.setYaw(0);
          Map.swerve.drive(0,-driveToPid.calculate(gamePieceY()),/*-targetPid.calculate(gamePieceX())*/0);
      }
    }
  }

  /**
   * Calculates the drive-to value for the game piece.
   *
   * @return The drive-to value for the game piece.
   */
  public static double driveToGamePiece() {
    if (Map.lightStop.get()){
      return 0;
    }else{
    return -driveToPid.calculate(gamePieceY());
    }
  }
}
