package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;

public class IntakeRollersIOReal implements IntakeRollersIO {
  private final TalonSRX topIntake = new TalonSRX(19);
  private final TalonSRX bottomIntake = new TalonSRX(20);

  // public static DigitalInput lightStop = new DigitalInput(2);
  // public static DigitalInput intakeStop = new DigitalInput(3);
  // private final double leaderVelocity = intakeTop.getVelocity();
  // private final StatusSignal<Double> leaderAppliedVolts = intakeTop.getMotorVoltage();

  public IntakeRollersIOReal() {
    topIntake.setNeutralMode(NeutralMode.Brake);
    bottomIntake.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    // TODO Auto-generated method stub
    IntakeRollersIO.super.updateInputs(inputs);
  }
  @Override
  public void setRollers(double topSpeed, double bottomSpeed) {
    topIntake.set(ControlMode.PercentOutput, topSpeed);
    bottomIntake.set(ControlMode.PercentOutput, bottomSpeed);
  }

  @Override
  public void stop() {
    topIntake.set(ControlMode.PercentOutput, 0);
    bottomIntake.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    //  extend.getConfigurator().apply(config);
  }
}
