package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto {
    public static double getStartTime() {
        double startTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("startTime", startTime);
        return startTime;
    }

    public static void runAuto(double startTime) {

        PIDController twistPID = new PIDController(.007, 0, 0);
        double autoYaw = Swerve.gyro.getYaw();
        double rawTime = Timer.getFPGATimestamp();

        double time = rawTime - startTime;
        double exicutionA = 3 - time;
        double exicutionB = 90;
        SmartDashboard.putNumber("startTime", startTime);
        SmartDashboard.putNumber("rawTime", rawTime);
        SmartDashboard.putNumber("time", time);
        if (time <= 3) {
            Map.swerve.auto(.2 * exicutionA, 0, 0);
        } else if (time > 3 && time <= 6) {
            Map.swerve.drive(0, 0, twistPID.calculate(exicutionB, autoYaw));

        } else {
            Map.swerve.drive(0, 0, 0);
        }
    }

    public static void runAutoDrive(double feetTarget, double angle) {
        PIDController speedPID = new PIDController(.0001, 0.0000001, 0.0000000);
        double tickTarget = feetTarget * 12000;
        double rawDistanceBL = Map.backLeft.driveMotor.getSelectedSensorPosition();
        
        Map.backLeft.drive(speedPID.calculate(Math.abs(tickTarget)-Math.abs(rawDistanceBL)),(angle + Swerve.blOffset));
        Map.backRight.drive(speedPID.calculate(Math.abs(tickTarget)-Math.abs(rawDistanceBL)),angle+ Swerve.brOffset);
        Map.frontLeft.drive(speedPID.calculate(Math.abs(tickTarget)-Math.abs(rawDistanceBL)),angle+Swerve.flOffset);
        Map.frontRight.drive(speedPID.calculate(Math.abs(tickTarget)-Math.abs(rawDistanceBL)),angle+Swerve.frOffset);
    }
}
