package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.RaspberryPi;

/**
 * This class represents the launcher of the robot.
 * It provides methods to control the launcher.
 */
public class Launcher {
   public static TalonFX launcherPivot = new TalonFX(17);
  static boolean goTo45 = false;
  static boolean launcherReady = false;

  public static PIDController launcherPID = new PIDController(.0042, 0, 0);
  public static PIDController pivotPid = new PIDController(.0039, 0.0000, 0);

  public static boolean toggleTarget = false;
  public static double angleTuner = 0;

  /**
   * Initializes the launcher by setting the selected feedback sensor.
   */

  public static void init() {
    launcherPivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    Map.rightLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    Map.leftLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    // intakeRight is what the pivot encoder is wired to.
    Map.intakeRight.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    launcherPivot.setSelectedSensorPosition(0);
    launcherPivot.setNeutralMode(NeutralMode.Brake);
    Map.rightLauncher.setInverted(true);
    goTo45 = false;
    angleTuner = 0;

  }

  public static void disable(boolean button) {
    goTo45 = false;
    if (button) {
      launcherPivot.setNeutralMode(NeutralMode.Coast);
    }

    else {
      launcherPivot.setNeutralMode(NeutralMode.Brake);
    }
  }

  public static double manualAngleTuner(int POV) {
    if (Map.pivotTop.get() == false) {
      angleTuner = 0;
    } else if (POV == 0 || POV == 315 || POV == 45) {
      angleTuner = angleTuner - 10;
      Timer.delay(.05);
    } else if (POV == 180 || POV == 255 || POV == 135) {
      angleTuner = angleTuner + 10;
      Timer.delay(.05);
    } else if (POV == 90 || POV == 270) {
      angleTuner = 0;
    }
    SmartDashboard.putNumber("angle Tuner", angleTuner);
    return angleTuner;
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

  public static void run(boolean fourtyFive,boolean sixty, double tuner, boolean autoFourtyFive) {
    
    if(autoFourtyFive){
      goTo45 = true;
    }
    else if (fourtyFive) {
      goTo45 = !goTo45;

    } if(sixty){
         launcherPivot.set(ControlMode.PercentOutput,
            pivotPid.calculate(Map.intakeRight.getSelectedSensorPosition(), 1420));
            if(Map.pivotTop.get()==false){
                 launcherPivot.set(ControlMode.PercentOutput,0);

            }
    }
   else if (goTo45) {
    if (Map.intakeRight.getSelectedSensorPosition()<1050){
      goTo45 = false;
       launcherPivot.set(ControlMode.PercentOutput, 0);
    }
     else if (Map.leftElevator.getSelectedSensorPosition() < -20000) {
        launcherPivot.set(ControlMode.PercentOutput,
            // jut in case: -36900 is motor tick to go back to.
        pivotPid.calculate(Map.intakeRight.getSelectedSensorPosition(), 1100));
      } else {
        launcherPivot.set(ControlMode.PercentOutput,
            pivotPid.calculate(Map.intakeRight.getSelectedSensorPosition(), (1235 + tuner)));
      }
    } else {
      launcherPivot.set(ControlMode.PercentOutput, 0);
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