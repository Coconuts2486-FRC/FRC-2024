package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  public static class IntakeIORollersInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerDutyCycle(
      double staticPercentTop,
      double staticPercentBottom,
      double manualIn,
      double manualOut,
      boolean lightStop) {}

  public default void stop() {}

  public default void setVoltage(double volts){}
  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
