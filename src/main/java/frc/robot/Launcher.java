package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.RaspberryPi;

/**
 * This class represents the launcher of the robot.
 * It provides methods to control the launcher.
 */
public class Launcher {
  static boolean goTo45 = false;
  static boolean launcherReady = false;
  public static PIDController launcherPID = new PIDController(.00008, 0.00000, 0.00000);
  public static PIDController launcherPID2 = new PIDController(.000082, 0.00000, 0.00000);

  public static boolean toggleTarget = false;
  public static double angleTuiner = 0;

  /**
   * Initializes the launcher by setting the selected feedback sensor.
   */

  public static void init() {
    Map.launcherPivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    Map.rightLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    Map.leftLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    Map.launcherPivot.setSelectedSensorPosition(0);
    Map.launcherPivot.setNeutralMode(NeutralMode.Brake);
    Map.rightLauncher.setInverted(true);
    goTo45 = false;

    Map.launcherPivot.config_kP(0, 0.0055);
    Map.launcherPivot.config_kI(0, 0.0000);
    Map.launcherPivot.config_kD(0, 0.000001);

  }

  public static void disable(boolean button) {
    goTo45 = false;
    if (button) {
      Map.launcherPivot.setNeutralMode(NeutralMode.Coast);
    }

    else {
      Map.launcherPivot.setNeutralMode(NeutralMode.Brake);
    }
  }
  public static double manualAngleTuiner(){
    return 2;
  }

  /**
   * Powers up the launcher based on the button press.
   *
   * @param buttonPress The value of the button press.
   */
  

  /**
   * Calculates the angle of the launcher based on the distance.
   *
   * @param distance The distance to the target.
   * @return The angle of the launcher.
   */
  public static double calculateAngle(double distance) {
    // insert regression alg. Distance converted to Position.
    // reg alg
    double calculation = distance * 123 - distance * 123;

    double conversion = 2651;
    calculation = (calculation * conversion);

    if (calculation < 0) {
      calculation = 0;
    }
    return calculation;
  }

  public static void test(boolean button1, boolean button2, boolean button3, boolean button4, double axis, boolean red,
      boolean autoTrue, boolean autoZeroTrue) {

    double calculation;
    if (autoTrue) {
      goTo45 = true;
    } else if (button2) {
      goTo45 = !goTo45;
    }
    if (red) {
      calculation = calculateAngle(RaspberryPi.getTagZ4());
    } else if (red == false) {
      calculation = calculateAngle(RaspberryPi.getTagZ7());
    }
    if(Map.intakeStop.get()==false && Map.launcherPivot.getSelectedSensorPosition() >= -21000){
      Map.launcherPivot.set(ControlMode.PercentOutput,0);
    }
    else if (button1) {
      if (Map.pivotTop.get() == false) {
        Map.launcherPivot.setSelectedSensorPosition(0);
        Map.launcherPivot.set(ControlMode.PercentOutput, -0);
      } else if (Map.pivotTop.get() == true) {
        Map.launcherPivot.set(ControlMode.PercentOutput, .5);
      }
    }

    else if (button3) {
      Map.launcherPivot.set(ControlMode.PercentOutput, axis * .3);
      if (axis > -.022) {
        Map.launcherPivot.set(ControlMode.PercentOutput, -.022);
      }
    }
else if(autoZeroTrue) {
   Map.launcherPivot.set(ControlMode.PercentOutput,
            launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(), 0));
}
    else if (goTo45) {
      if (Map.leftElevator.getSelectedSensorPosition() < -20000) {
        Map.launcherPivot.set(ControlMode.PercentOutput,
            launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(), -36900));

      } else if (button4) {
        if (red) {
          if (RaspberryPi.getTagZ4() < 90) {
            Map.launcherPivot.set(ControlMode.PercentOutput,
                launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(), -21900));
          }else if (RaspberryPi.getTagZ4() > 90){
           Map.launcherPivot.set(ControlMode.PercentOutput,
            launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(), -23900)); 
          }
        } else {
   if (RaspberryPi.getTagZ7() < 90) {
            Map.launcherPivot.set(ControlMode.PercentOutput,
                launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(), -21900));
          }else if (RaspberryPi.getTagZ7() > 90){
           Map.launcherPivot.set(ControlMode.PercentOutput,
            launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(), -23900)); 
          }

        }
      } else {
        Map.launcherPivot.set(ControlMode.PercentOutput,
            launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(), -21900));
      }
      if (Map.pivotTop.get() == false) {
        Map.launcherPivot.setSelectedSensorPosition(0);
      }

    }

    else {
      Map.launcherPivot.set(ControlMode.PercentOutput, 0);
    }

  }

  public static void launch(boolean button) {
    if (button) {
      Map.leftLauncher.set(TalonSRXControlMode.Velocity, 21000);
      Map.rightLauncher.set(TalonSRXControlMode.Velocity, 21000);
      SmartDashboard.putNumber("rightLaunch", Map.rightLauncher.getSelectedSensorVelocity());
      SmartDashboard.putNumber("leftLaunch", Map.leftLauncher.getSelectedSensorVelocity());
    } else {
      Map.leftLauncher.set(ControlMode.PercentOutput, .0);
      Map.rightLauncher.set(ControlMode.PercentOutput, -.0);
    }
  }

  public static void launchAuto(boolean button) {
    if (button) {
      Map.leftLauncher.set(ControlMode.Velocity, 21500);
      Map.rightLauncher.set(ControlMode.Velocity, 21500);
      SmartDashboard.putNumber("rightLaunch", Map.rightLauncher.getSelectedSensorVelocity());
      SmartDashboard.putNumber("leftLaunch", Map.leftLauncher.getSelectedSensorVelocity());
    } else {
      Map.leftLauncher.set(ControlMode.Velocity, 10000);
      Map.rightLauncher.set(ControlMode.Velocity, 10000);
    }
  }
}