package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  public Pivot(PivotIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0., 0.0);
        io.configurePID(0.0, 0.0, 0.0);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    Logger.recordOutput("Pivot/DistanceToSpeaker", getSpeakerDistance());
    SmartDashboard.putNumber("Speaker Distance", getSpeakerDistance());
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void holdPosition(double angle) {
    io.holdPosition(angle);
  }
  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Pivot/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public double pivotAngle() {

    return PivotIOInputsAutoLogged.positionDeg;
  }

  /******************************************************************************************/
  /** NOTE: These functions should probably end up somewhere else, but they're here for now */

  /**
   * Compute the distance to the SPEAKER AprilTag, as seen by PhotonVision
   *
   * <p>Returns the SPEAKER distance (along the floor) in inches. If the speaker tag is not visible,
   * this returns -999.9 inches!
   *
   * <p>NOTE: If we want the null result to return something other than -999.9, we can do that.
   */
  public double getSpeakerDistance() {

    // No tag information, return default value
    if (AprilTagVision.speakerPose == null) {
      return -999.9;
    }

    // Return the distance to the tag along the floor in inches
    return Units.metersToInches(
        Math.hypot(AprilTagVision.speakerPose.getX(), AprilTagVision.speakerPose.getY()));
  }

  /**
   * Compute the field-centric YAW to the SPEAKER AprilTag, as seen by PhotonVision
   *
   * <p>Returns the field-centric YAW to the SPEAKER in degrees. To aim the robot at the speaker,
   * set the robot YAW equal to this value. If the speaker tag is not visible, this returns -999.9
   * degrees!
   *
   * <p>NOTE: This function assumes "Always Blue Origin" convention for YAW, meaning that when
   * alliance is BLUE, 0º is away from the alliance wall, and when alliance is RED, 180º is away
   * from the alliance wall. A head-on speaker shot has the robot facing away from the alliance wall
   * (i.e., the shooter is on the back of the robot).
   *
   * <p>NOTE: If we want the null result to return something other than -999.9, we can do that.
   */
  // find others in Drive.java

}
