package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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
  public static PIDController launcherPID = new PIDController(.00008,0.00000,0.00000);
    public static PIDController launcherPID2 = new PIDController(.000082,0.00000,0.00000);
  
     public static boolean toggleTarget = false;
  /**
   * Initializes the launcher by setting the selected feedback sensor.
   */

   public static void init() {
    Map.launcherPivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
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
public static void disable(boolean button){
    if (button){      Map.launcherPivot.setNeutralMode(NeutralMode.Coast);}
    
    else{
            Map.launcherPivot.setNeutralMode(NeutralMode.Brake);
    }
}
  /**
   * Powers up the launcher based on the button press.
   *
   * @param buttonPress The value of the button press.
   */
  public static void powerUp(double buttonPress) {
    if (buttonPress >= .15) {
      Map.rightLauncher.set(ControlMode.PercentOutput, 1);
      Map.leftLauncher.set(ControlMode.PercentOutput, -1);
    } else {
      Map.rightLauncher.set(ControlMode.PercentOutput, 0);
      Map.leftLauncher.set(ControlMode.PercentOutput, 0);
    }
    if (Map.rightLauncher.getMotorOutputPercent() == 1) {
      if (Map.leftLauncher.getMotorOutputPercent() == -1) {
        launcherReady = true;
      } else {
        launcherReady = false;
      }
    } else {
      launcherReady = false;
    }
    SmartDashboard.putBoolean("Launcher Ready", launcherReady);
  }

  /**
   * Calculates the angle of the launcher based on the distance.
   *
   * @param distance The distance to the target.
   * @return The angle of the launcher.
   */
  public static double calculateAngle(double distance) {
    // insert regression alg. Distance converted to Position.
    //reg alg
    double calculation = distance * 123 - distance * 123;

    double conversion = 2651;
    calculation = (calculation * conversion);

    if (calculation<0){
        calculation = 0;
    }
    return calculation;
  }
 
    public static void test(boolean button1,boolean button2, boolean button3, double axis, boolean red) {
       
        double calculation;
   
      if (button2){
        goTo45 = !goTo45;
      }
        if (red){
            calculation = calculateAngle(RaspberryPi.getTagZ4());
        }else if (red==false){
            calculation = calculateAngle(RaspberryPi.getTagZ7());
        }
        if (button1) {
          if (Map.pivotTop.get()==false){
               Map.launcherPivot.setSelectedSensorPosition(0);
             Map.launcherPivot.set(ControlMode.PercentOutput,-0);
          }
            else if (Map.pivotTop.get()==true) {
                 Map.launcherPivot.set(ControlMode.PercentOutput,.5);
            }
        }


  else if(button3) {
              Map.launcherPivot.set(ControlMode.PercentOutput, axis*.3);
              if(axis >- .022){
                 Map.launcherPivot.set(ControlMode.PercentOutput, -.022);
              }
          }

       else if (goTo45) {
        if(Map.leftElevator.getSelectedSensorPosition()<-20000){
         Map.launcherPivot.set(ControlMode.PercentOutput, launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(), -36900));
       
        }else{
           Map.launcherPivot.set(ControlMode.PercentOutput, launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(), -23900));

        }
            if (Map.pivotTop.get()== false) {
                Map.launcherPivot.setSelectedSensorPosition(0);
            }
    

          } 
        
        else{
            Map.launcherPivot.set(ControlMode.PercentOutput,0);
        }

  }

     /**
   * Runs the launcher based on the left trigger value and the color of the target.
   *
   * @param leftTrigger The value of the left trigger.
   * @param red The color of the target.
   * @return The state of the launcher.
   */
  
  public static boolean run(double leftTrigger, boolean red) {
        //45 degrees. change this
        double calculatedAngle;
        // change this
        int upperLimit = 1;
        if (leftTrigger >= .15) {
            powerUp(leftTrigger);

            if (red) {
                calculatedAngle = Math.round(calculateAngle(RaspberryPi.getTagX4()));
                
            } else {
                calculatedAngle = Math.round(calculateAngle(RaspberryPi.getTagX7()));
            }
            if (calculatedAngle > upperLimit) {
                    calculatedAngle = upperLimit;
            }
            if (calculatedAngle < 0) {
                    calculatedAngle = 0;
            }
            if (Map.pivotTop.get()==false) {
                Map.launcherPivot.setSelectedSensorPosition(upperLimit);
            }  
    
            }
            else{
                //45 degrees. change this
                calculatedAngle = -21000;
            }
                // change number later
                //checks if it is ready

                Map.launcherPivot.set(ControlMode.PercentOutput, launcherPID.calculate(Map.launcherPivot.getSelectedSensorPosition(),calculatedAngle));
                if (Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition())-Math.abs(calculatedAngle))<800){
                    return true;
                }
                else {
                    return false;
                }
            }
  
            
        
    

    
  /**
   * Automatically shoots the game piece based on the color of the target.
   *
   * @param red The color of the target.
   * @return The state of the launcher.
   */
    // shooting for auto
    public static boolean autoShoot(boolean red) {
        boolean gamePiecePresent;
        if (Map.lightStop.get()) {
            gamePiecePresent = true;
        } else {
            gamePiecePresent = false;
        }

    boolean spunUp = false;
    boolean alligned = false;
    boolean aimed = run(.5, red);
    boolean ready = false;
    boolean shot = false;
    // change velocity to a higher number
    if (
      Map.rightLauncher.getSelectedSensorVelocity() > 8 &&
      Map.leftLauncher.getSelectedSensorVelocity() > 8
    ) {
      spunUp = true;
    } else {
      spunUp = false;
    }
    if (red) {
      if (RaspberryPi.getTagX4() < 3 && RaspberryPi.getTagX4() > 0) {
        alligned = true;
      } else alligned = false;
    } else {
      if (RaspberryPi.getTagX7() < 3 && RaspberryPi.getTagX7() > 0) {
        alligned = true;
      } else alligned = false;
    }
    if (spunUp && aimed && alligned) if (ready == true) {}
    return shot;
  }
}
