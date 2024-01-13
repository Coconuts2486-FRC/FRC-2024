package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Map;

public class AutoPath {
    public static void driveOutPickUpShoot(){
        PIDController autoPID = new PIDController(0, 0, 0);
        double x = Map.odometry.calculatePosition()[0];
        double y = Map.odometry.calculatePosition()[1];
        double targetX=0;
        double targetY=10;
        double difX = targetX-x;
        double difY = targetY-y;
        Map.swerve.drive(autoPID.calculate(x,targetX), autoPID.calculate(y,targetY), 0);
        if (Math.abs(difX)<1000 && Math.abs(difY)<1000) {

        }
    }
}
