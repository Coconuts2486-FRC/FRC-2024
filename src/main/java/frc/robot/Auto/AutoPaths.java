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
            if (RaspberryPi.gamePieceY()<=4||RaspberryPi.gamePieceY()>=60){
            AutoMethod.autoDriveY(2000);
             a = a++;
            } else if (RaspberryPi.gamePieceY()>4&&RaspberryPi.gamePieceY()<60){
                a = a++;
            }
        }

    }
}
