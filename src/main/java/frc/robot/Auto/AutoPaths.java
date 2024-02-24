package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Intake;
import frc.robot.Launcher;
import frc.robot.Map;
import frc.robot.Swerve;
import frc.robot.Vision.RaspberryPi;

public class AutoPaths {
    private static int a;
    public static double autoYaw;

    // Change names later
    public static void autoInit(){
        a = 0;
    }

    public static void autoOne(boolean red) {
    
        if (a == 0) {
            Intake.test(false,false, false, false,false);
            Launcher.test(false, false, true,0,red,false);
            Map.swerve.realignToField(true);
            // autoYaw = Swerve.gyro.getYaw();
            a = a++;
        }
       else if (a==1){
            
         Launcher.test(false, false, false,0,red,true);
          Intake.test(true,false, false, false,true);
        RaspberryPi.targetGamePiece(false, true, false);
            if (Map.lightStop.get()){
                a=a++;
            }
        } 
        else if (a == 2){
            Intake.test(false, false, false, false, true);
             Launcher.test(false, false, false,0,red,true);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true,0,red));
            if (Math.abs(RaspberryPi.getTagX4())<5||Math.abs(RaspberryPi.getTagX7())<5){
                a=a++;
            }
        }else if (a ==3){
            Intake.test(false, false, true, false, true);
             Launcher.test(false, false, false,0,red,true);
            Launcher.launchAuto(true);
        }

    }
}
