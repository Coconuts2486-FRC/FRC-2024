package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX elevatorMotor = new TalonFX(22, "drive");

  public ElevatorIOReal() {
    var motorConfig = new MotorOutputConfigs();
    motorConfig.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void setDutyCycle(double percent) {
    elevatorMotor.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void stop() {
    elevatorMotor.setControl(new DutyCycleOut(0));
  }
}
