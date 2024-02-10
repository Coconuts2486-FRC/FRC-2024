package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher {
    static boolean launcherReady = false;
    static PIDController pivotLauncher = new PIDController(.5, 0.001, 0);
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

    public static void aim(int leftTrigger, int pivotPosistion) {
        if (leftTrigger >= .15) {
            powerUp(leftTrigger);
            if (Map.pivotTop.DIO()) {
            } else if (Map.pivotBottom.DIO()) {
            } else {
                Map.launcherPivot.set(ControlMode.Position, pivotPosistion);
            }
        }
    }

    public static void finalShoot(boolean rightTrigger) {
        Intake.intakeSpin(rightTrigger);
    }
}
// !Code :)