package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeIOReal implements IntakeIO {
  private static final TalonSRX intakeTop = new TalonSRX(0);
  private static final TalonSRX intakeBottom = new TalonSRX(1);

  private final double leaderPosition = intakeTop.getSelectedSensorPosition();
  // private final double leaderVelocity = intakeTop.getVelocity();
  // private final StatusSignal<Double> leaderAppliedVolts = intakeTop.getMotorVoltage();

  public IntakeIOReal() {
    intakeTop.setNeutralMode(NeutralMode.Brake);
    intakeBottom.setNeutralMode(NeutralMode.Brake);
  }
}
