package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Map;
import frc.robot.Swerve;
import frc.robot.Vision.RaspberryPi;

public class AutoPaths {
    public static double autoYaw;

    // Change names later
    public static void autoOne(boolean red) {
        int a = 0;
        if (a == 0) {
            Map.swerve.realignToField(true);
            autoYaw = Swerve.gyro.getYaw();
            a = a++;
        }
        if (a==2){
            if (RaspberryPi.gamePieceZ()<=4||RaspberryPi.gamePieceZ()>=60){
            AutoMethod.autoDriveY(2000);
             a = a++;
            } else if (RaspberryPi.gamePieceZ()>4&&RaspberryPi.gamePieceZ()<60){
                a = a++;
            }
        }

    }
}
