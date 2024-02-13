package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.RaspberryPi;

public class Launcher {
    static boolean launcherReady = false;
    public static PIDController pivotLauncher = new PIDController(0.5, 0.001, 0);
    public static void shootMotorInit() {
        Map.rightLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        Map.leftLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    }

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

    public static double calculateAngle(double distance) {
        // insert regression alg. Distance converted to Position.
        //reg alg
       double calculation = distance * 123-distance*123; 
        
        double conversion = 2651;
        calculation = (calculation*conversion);
        return calculation;
    }

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
            if (Map.pivotTop.DIO()) {
                Map.launcherPivot.setSelectedSensorPosition(upperLimit);
            }  
    
            }
            else{
                //45 degrees. change this
                calculatedAngle = 45;
            }
                // change number later
                //checks if it is ready

                Map.launcherPivot.set(ControlMode.PercentOutput, pivotLauncher.calculate(Map.launcherPivot.getSelectedSensorPosition(),calculatedAngle));
                if (Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition())-Math.abs(calculatedAngle))<800){
                    return true;
                }
                else {
                    return false;
                }
            }
  
            
        
    

    

    // shooting for auto
    public static boolean autoShoot(boolean red) {
        boolean gamePiecePresent;
        if (Map.lightStop.DIO()) {
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
        if (Map.rightLauncher.getSelectedSensorVelocity() > 8 && Map.leftLauncher.getSelectedSensorVelocity() > 8) {
            spunUp = true;
        } else {
            spunUp = false;
        }
        if (red) {
            if (RaspberryPi.getTagX4() < 3 && RaspberryPi.getTagX4() > 0) {
                alligned = true;
            } else
                alligned = false;
        } else {
            if (RaspberryPi.getTagX7() < 3 && RaspberryPi.getTagX7() > 0) {
                alligned = true;
            } else
                alligned = false;
        }
        if(spunUp&&aimed&&alligned)
        if (ready == true) {

        }
        return shot;
    }
}