package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOReal implements IntakeIO {
  private final TalonSRX intakeTop = new TalonSRX(20);
  private final TalonSRX intakeBottom = new TalonSRX(19);
  private final TalonFX extend = new TalonFX(15);
  private final double leaderPosition = intakeTop.getSelectedSensorPosition();
  private final DigitalInput lightStop = new DigitalInput(0);
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
  public void setExtendPosition(double position) {
    extend.setControl(new PositionDutyCycle(10));
  }

  @Override
  public void setRollerDutyCycle(double percent){

  }

  @Override
  public void stop(){

  }
}
