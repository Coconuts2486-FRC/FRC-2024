package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX extend = new TalonFX(18);

  // public static DigitalInput lightStop = new DigitalInput(2);
  // public static DigitalInput intakeStop = new DigitalInput(3);
  // private final double leaderVelocity = intakeTop.getVelocity();
  // private final StatusSignal<Double> leaderAppliedVolts = intakeTop.getMotorVoltage();

  public IntakeIOReal() {
    var brake = new MotorOutputConfigs();
    brake.NeutralMode = NeutralModeValue.Brake;
    extend.getConfigurator().apply(brake);

    var ramp = new ClosedLoopRampsConfigs();
    ramp.withTorqueClosedLoopRampPeriod(2);
    extend.getConfigurator().apply(ramp);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    IntakeIOInputs.pose = extend.getPosition().getValueAsDouble();
  }

  @Override
  public void setExtendPosition(double position) {
    extend.setControl(new PositionDutyCycle(position));
  }

  @Override
  public void stop() {
    extend.setControl(new DutyCycleOut(0));
  }

  @Override
  public void retract() {
    // if (limit) {
    //   extend.setControl(new DutyCycleOut(0));
    //   extend.setPosition(0);
    // } else
    if (extend.getPosition().getValueAsDouble() > 4.8) {
      extend.setControl(new DutyCycleOut(-.8));
    } else {
      extend.setControl(new DutyCycleOut(-.3));
    }
  }

  @Override
  public void setPose(int position) {
    extend.setPosition(0);
  }

  @Override
  public void coastTrue(boolean coast) {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = coast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    extend.getConfigurator().apply(config);
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
