package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Intake;
import frc.robot.Launcher;
import frc.robot.Map;
import frc.robot.Swerve;
import frc.robot.Vision.RaspberryPi;

public class AutoPaths {
    private static int a = 0;
    public static double autoYaw;

    // Change names later
    public static void autoInit(){

        a = 0;
    }

    public static void autoOne(boolean red) {
    
        if (a == 0) {
            Intake.test(false,false, false, false,false,0,0);
            Launcher.test(false, false, false,0,red,false);
            Map.swerve.realignToField(true);
            // autoYaw = Swerve.gyro.getYaw();
            a = a+1;
        }
       else if (a==1){
            
         Launcher.test(false, false, false,0,red,true);
          Intake.test(true,false, false, false,true,0,0);
        RaspberryPi.targetGamePiece( true,false);
            if (Map.lightStop.get()){
                a=a+1;
            }
        } 
        else if (a == 2){
            Intake.test(false, false, false, false, true,0,0);
             Launcher.test(false, false, false,0,red,true);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true,0,red));
             Launcher.launchAuto(true);
            if (Math.abs(RaspberryPi.getTagX4())<5||Math.abs(RaspberryPi.getTagX7())<5){
                   

                a=a+1;
            }
        }else if (a ==3){
            Intake.test(false, false, true, false, true,0,0);
             Launcher.test(false, false, false,0,red,true);
            Launcher.launchAuto(true);
                  Map.swerve.drive(0, 0, 0);
        }

    }
}
