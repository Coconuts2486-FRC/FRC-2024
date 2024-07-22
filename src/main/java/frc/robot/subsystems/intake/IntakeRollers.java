package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  private final IntakeRollersIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  // private final SimpleMotorFeedforward ffModel;
  // private final SysIdRoutine sysId;

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    // switch (Constants.currentMode) {
    //   case REAL:
    //     ffModel = new SimpleMotorFeedforward(0., 0.0);
    //     io.configurePID(5.101, 0.0, 0.001);
    //     break;
    //   case REPLAY:
    //     ffModel = new SimpleMotorFeedforward(0.1, 0.05);
    //     io.configurePID(1.0, 0.0, 0.0);
    //     break;
    //   case SIM:
    //     ffModel = new SimpleMotorFeedforward(0.0, 0.03);
    //     io.configurePID(0.5, 0.0, 0.0);
    //     break;
    //   default:
    //     ffModel = new SimpleMotorFeedforward(0.0, 0.0);
    //     break;
  }

  //     sysId =
  //         new SysIdRoutine(
  //             new SysIdRoutine.Config(
  //                 null,
  //                 null,
  //                 null,
  //                 (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
  //             new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  //   }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  /** Run open loop at the specified voltage. */

  /** Run closed loop at the specified velocity. */

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  public void setRollers(
      double staticPercentTop,
      double staticPercentBottom,
      double manualIn,
      double manualOut,
      boolean lightstop) {
    io.setRollerDutyCycle(staticPercentTop, staticPercentBottom, manualIn, manualOut, lightstop);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  //   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //     return sysId.quasistatic(direction);
  //   }

  /** Returns a command to run a dynamic test in the specified direction. */
  //   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //     return sysId.dynamic(direction);
  //   }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
