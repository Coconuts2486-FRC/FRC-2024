package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double velocityRaw = 0.0;
    public double appliedVolts = 0.0;
    public boolean limitPressed = false;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  public default void setDutyCycle(double percent) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
