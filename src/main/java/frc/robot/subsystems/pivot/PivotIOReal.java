package frc.robot.subsystems.pivot;

import static frc.robot.subsystems.pivot.PivotIO.PivotIOInputs.positionDeg;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

public class PivotIOReal implements PivotIO {

  private final TalonFX pivot = new TalonFX(0);
  private final CANcoder pivotCANcoder = new CANcoder(1);
  private final PIDController pivotPID = new PIDController(.056, 0.0000, 0.000001);

  private final StatusSignal<Double> pivotPosition = pivotCANcoder.getAbsolutePosition();
  private final StatusSignal<Double> pivotAppliedVolts = pivot.getMotorVoltage();
  private final StatusSignal<Double> pivotCurrent = pivot.getSupplyCurrent();

  public PivotIOReal() {
    var pivotTalonConfig = new TalonFXConfiguration();
    pivotTalonConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    pivotTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    pivotTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivot.getConfigurator().apply(pivotTalonConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, pivotPosition, pivotAppliedVolts, pivotCurrent);
    pivot.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(pivotPosition, pivotAppliedVolts, pivotCurrent);
    positionDeg = pivotPosition.getValueAsDouble() * 360;
    //   inputs.velocityRadPerSec =
    //       Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {pivotCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    pivot.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocity, double ffVolts) {
    pivot.setControl(new VelocityDutyCycle(velocity));
  }

  @Override
  public void setDutyCycle(double percentOut) {
    pivot.setControl(new DutyCycleOut(percentOut));
  }

  @Override
  public void holdPosition(double angle){
    double calculate = pivotPID.calculate(PivotIOInputs.positionDeg, angle);
    pivot.setControl(new DutyCycleOut(calculate));
  }

  @Override
  public void stop() {
    pivot.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    pivot.getConfigurator().apply(config);
  }
}
