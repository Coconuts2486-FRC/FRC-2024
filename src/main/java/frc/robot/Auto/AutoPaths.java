package frc.robot.Auto;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;
import frc.robot.Launcher;
import frc.robot.Map;
import frc.robot.Swerve;
import frc.robot.Vision.RaspberryPi;

public class AutoPaths {
    

    public static int a = 0;
    public static double autoYaw;
    public static double startTime;


    // Change names later
    public static void autoInit(){
        startTime =  0;
        a = 0;
        Map.leftLauncher.setSelectedSensorPosition(0);
                Map.rightLauncher.setSelectedSensorPosition(0);
        
    }

    public static void autoOne(boolean red, double start) {
    
        if (a == 0) {
            Intake.test(false,false, false, false,false,false,0,0,false,red);
            Launcher.test(false, false, false,false,0,red,false,true);
            Map.swerve.realignToField(true);
                       
         
            // autoYaw = Swerve.gyro.getYaw();
            a = a+2;
        }

         if (a==1){
                
                Launcher.launch(true);       
                                  
         Launcher.test(false, false, false,false,0,red,false,true);
          Intake.test(false,false, true,false, false,false,0,0,true,red);
          
             if(Map.leftLauncher.getSelectedSensorVelocity()>21560){
                     
             Timer.delay(.4);
                    a=a+1;    
            }
                
         
              }
              
       
        
       else if (a==2){
            
         Launcher.test(false, false, false,false,0,red,true,false);
          Intake.test(true,false, false,false, false,true,0,0,false,red);
        RaspberryPi.targetGamePiece( true,false);
            Launcher.launchAuto(false);
            if (Map.lightStop.get()){
                a=a+1;
            }
        } 
        else if (a == 3){
            Intake.test(false, false, false,false, false, true,0,0,false,red);
             Launcher.test(false, false, false,false,0,red,true,false);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true,0,red));
             Launcher.launchAuto(true);
            if (Math.abs(RaspberryPi.getTagX4())<5||Math.abs(RaspberryPi.getTagX7())<5){
                   

                a=a+1;
            }
        }else if (a ==4){
            Intake.test(false, false, true,false, false, true,0,0,false,red);
             Launcher.test(false, false, false,false,0,red,true,false);
            Launcher.launchAuto(true);
                  Map.swerve.drive(0, 0, 0);
            if(Map.leftLauncher.getSelectedSensorVelocity()>21560){
                startTime=Timer.getFPGATimestamp();
            }
                  if (Map.leftLauncher.getSelectedSensorVelocity()>21560){
                    Timer.delay(.4);
               a=a+1;     
              }
              
        } else if(a==5);
            {
                  Intake.test(false, false, false, false,false, false,0,0,false,red);
             Launcher.test(false, false, false,false,0,red,false,false);
             Launcher.launch(false);
   
                  Map.swerve.drive(0, 0, 0);

    }
}
}
