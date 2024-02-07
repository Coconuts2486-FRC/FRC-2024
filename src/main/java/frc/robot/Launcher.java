package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.RaspberryPi;

public class Launcher {
    static boolean launcherReady = false;

    public static void shootMotorInit() {
        Map.rightLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        Map.leftLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        Map.rightLauncher.config_kF(0, 0.048);
        Map.leftLauncher.config_kF(0, 0.048);
        Map.rightLauncher.config_kP(0, 0.4);
        Map.leftLauncher.config_kP(0, 0.4);
        Map.rightLauncher.config_kI(0, 0.001);
        Map.leftLauncher.config_kI(0, 0.001);
        Map.rightLauncher.config_IntegralZone(0, 300);
        Map.leftLauncher.config_IntegralZone(0, 300);
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
        double angle = 0;
        return angle;
    }

    public static boolean aim(double leftTrigger, boolean red) {
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
            }  if (Map.pivotBottom.DIO()) {
                Map.launcherPivot.setSelectedSensorPosition(0);
    
            }
                // change number later
                Map.launcherPivot.set(ControlMode.Position, calculatedAngle);
                if (Map.launcherPivot.getSelectedSensorPosition()==calculatedAngle){
                    return true;
                }
                else {
                    return false;
                }
            }
            else {
                return false;
            }
        }
    

    public static void manualShoot(boolean rightTrigger) {
        Intake.intakeSpin(rightTrigger);
    }

    // shooting for auto
    public static boolean autoShoot(boolean red) {
        boolean gamePiecePresent;
        if (Map.gamepieceStop.DIO()) {
            gamePiecePresent = true;
        } else {
            gamePiecePresent = false;
        }

        boolean spunUp = false;
        boolean alligned = false;
        boolean aimed = aim(.5, red);
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