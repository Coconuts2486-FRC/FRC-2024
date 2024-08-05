package frc.robot.subsystems.sma;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class SmaIntakeRollers extends SubsystemBase {

  private static TalonSRX intTopRoller;
  private static TalonSRX intBottomRoller;

  public SmaIntakeRollers() {

    intTopRoller = new TalonSRX(19);
    intBottomRoller = new TalonSRX(20);
  }

  public void autoShot(double shotIntake) {
    intTopRoller.set(ControlMode.PercentOutput, shotIntake);
    intBottomRoller.set(ControlMode.PercentOutput, shotIntake);
  }

  public static void lightstop(boolean lightstop) {

    LoggedDashboardBoolean lightStop = new LoggedDashboardBoolean("Lightstop Triggered", lightstop);
  }

  public void stop() {}

  // call with autoShot.autoShotFunction(0);
}
