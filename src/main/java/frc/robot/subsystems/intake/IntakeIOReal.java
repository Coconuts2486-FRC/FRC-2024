package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  private final TalonSRX intakeTop = new TalonSRX(20);
  private final TalonSRX intakeBottom = new TalonSRX(19);
  private final TalonFX extend = new TalonFX(18);
  private final double leaderPosition = intakeTop.getSelectedSensorPosition();
  // public static DigitalInput lightStop = new DigitalInput(2);
  // public static DigitalInput intakeStop = new DigitalInput(3);
  // private final double leaderVelocity = intakeTop.getVelocity();
  // private final StatusSignal<Double> leaderAppliedVolts = intakeTop.getMotorVoltage();

  public IntakeIOReal() {
    var brake = new MotorOutputConfigs();
    brake.NeutralMode = NeutralModeValue.Brake;
    intakeTop.setNeutralMode(NeutralMode.Brake);
    intakeBottom.setNeutralMode(NeutralMode.Brake);
    extend.getConfigurator().apply(brake);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO Auto-generated method stub
    IntakeIO.super.updateInputs(inputs);
  }

  @Override
  public void setExtendPosition(int position) {
    extend.setControl(new PositionDutyCycle(position));
  }

  @Override
  public void setRollerDutyCycle(
      double staticPercent, double manualIn, double manualOut, boolean lightStop) {
    // System.out.println(lightStop);
    if (lightStop == false) {
      intakeTop.set(ControlMode.PercentOutput, staticPercent);
      intakeBottom.set(ControlMode.PercentOutput, staticPercent);
    } else {
      intakeTop.set(ControlMode.PercentOutput, manualIn - manualOut);
      intakeBottom.set(ControlMode.PercentOutput, manualIn - manualOut);
    }
  }

  @Override
  public void stop() {
    intakeTop.set(ControlMode.PercentOutput, 0);
    intakeBottom.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void retract(boolean limit) {
    if (limit) {
      extend.setControl(new DutyCycleOut(0));
      extend.setPosition(0);
    } else {
      extend.setControl(new DutyCycleOut(-.12));
    }
  }

  @Override
  public void setPose(int position) {
    extend.setPosition(0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    extend.getConfigurator().apply(config);
  }
}
