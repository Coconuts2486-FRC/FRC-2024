package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeRollersIOReal implements IntakeRollersIO {
  private final TalonSRX intakeTop = new TalonSRX(20);
  private final TalonSRX intakeBottom = new TalonSRX(19);
  // private final double leaderPosition = intakeTop.getSelectedSensorPosition();

  public IntakeRollersIOReal() {

    intakeTop.setNeutralMode(NeutralMode.Brake);
    intakeBottom.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void stop() {
    intakeTop.set(ControlMode.PercentOutput, 0);
    intakeBottom.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void setRollerDutyCycle(
      double staticPercentTop,
      double staticPercentBottom,
      double manualIn,
      double manualOut,
      boolean lightStop) { // System.out.println(lightStop);
    if (lightStop == false) {
      intakeTop.set(ControlMode.PercentOutput, staticPercentTop);
      intakeBottom.set(ControlMode.PercentOutput, staticPercentBottom);
    } else {
      intakeTop.set(ControlMode.PercentOutput, manualIn - manualOut);
      intakeBottom.set(ControlMode.PercentOutput, manualIn - manualOut);
    }
  }
  @Override
  public void setVoltage(double volts) {
     // intakeTop.set(ControlMod)
  }

  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    // extend.getConfigurator().apply(config);
  }
}
