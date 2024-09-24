package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX elevatorMotor = new TalonFX(22, "drive");
  private final StatusSignal<Double> elevatorPosition = elevatorMotor.getPosition();
  private final StatusSignal<Double> elevatorVelocity = elevatorMotor.getPosition();
  // private final DigitalInput topLimit = new DigitalInput(1);
  // private BooleanSupplier limit;

  public ElevatorIOReal() {
    var motorConfig = new MotorOutputConfigs();
    motorConfig.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity);
    // limit = topLimit::get;
    inputs.velocityRaw = elevatorVelocity.getValueAsDouble();
    // inputs.limitPressed = limit.getAsBoolean();
    // inputs.velocityRadPerSec =
    //     Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    // inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    // inputs.currentAmps =
    //     new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setDutyCycleUp(double percent) {
    if (elevatorMotor.getPosition().getValueAsDouble() <= -82) {
      elevatorMotor.setControl(new DutyCycleOut(-.5));
    } else {
      elevatorMotor.setControl(new DutyCycleOut(percent));
    }
  }

  @Override
  public void setDutyCycleDown(double percent) {
    if (elevatorMotor.getPosition().getValueAsDouble() >= -10) {
      elevatorMotor.setControl(new DutyCycleOut(.5));
    } else {
      elevatorMotor.setControl(new DutyCycleOut(percent));
    }
  }

  @Override
  public void setDutyCycle(double percent) {
    elevatorMotor.setControl(new DutyCycleOut(percent));
  }

  @Override
  public void setPose(double pose) {
    elevatorMotor.setPosition(pose);
  }

  @Override
  public void coastTrue(boolean coast) {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = coast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    elevatorMotor.getConfigurator().apply(config);
  }

  @Override
  public void stop() {
    elevatorMotor.stopMotor();
  }
}
