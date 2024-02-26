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
       
        
    }

    public static void autoOne(boolean red) {
       
        if (a == 0) {
            Intake.test(false,false, false, false,false,false,0,0,false,red);
            Launcher.test(false, false, false,false,0,red,false,true);
            Map.swerve.realignToField(true);
                       
         
            autoYaw = Swerve.gyro.getYaw();
            a = a+1;
        }

        else if (a==1){
                Launcher.launch(true);
           
                                  
        Launcher.test(false, false, false,false,0,red,false,true);
         Intake.test(false,false, true,false, false,false,0,0,true,red);
          
            if(Map.leftLauncher.getSelectedSensorVelocity()>19560){
              Map.intakeLeft.set(ControlMode.PercentOutput,1); 
                     Map.intakeRight.set(ControlMode.PercentOutput,-1);             
            Timer.delay(.5);
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
             Launcher.test(false, false, false,true,0,red,false,false);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true,0,red));
             Launcher.launchAuto(true);
            if (Math.abs(RaspberryPi.getTagX4())<7||Math.abs(RaspberryPi.getTagX7())<7){
                   

                a=a+1;
            }
        }else if (a ==4){
            Intake.test(false, false, true,false, false, true,0,0,false,red);
             Launcher.test(false, false, false,true,0,red,false,false);
            Launcher.launchAuto(true);
                  Map.swerve.drive(0, 0, 0);
            if(Map.leftLauncher.getSelectedSensorVelocity()>21560){
                startTime=Timer.getFPGATimestamp();
            }
                  if (Map.leftLauncher.getSelectedSensorVelocity()>20560){
                        Map.intakeLeft.set(ControlMode.PercentOutput,1); 
                     Map.intakeRight.set(ControlMode.PercentOutput,-1);           
                    Timer.delay(.4);
               a=a+1;     
              }
              
         } 
         //else if (a==6){
        //                  Launcher.test(false, false, false,false,0,red,true,false);
        //                    Intake.test(false, false, true,false, false, true,0,0,false,red);
        //    Map.swerve.drive(0, 0, AutoMethod.autoRotate(Map.swerve.gyro.getYaw(), 90, red));
        //     Timer.delay(1);
        // }
        
        else if(a==5)   {
                  Intake.test(false, false, false, false,false, true,0,0,false,red);
             Launcher.test(false, false, false,false,0,red,false,true);
             Launcher.launch(false);
   
                  Map.swerve.drive(0, 0, 0);

    }
}

public static void auto2(){
    if (a == 0) {
            Intake.test(false,false, false, false,false,false,0,0,false,true);
            Launcher.test(false, false, false,false,0,true,false,true);
            Map.swerve.realignToField(true);
                       
         
            // autoYaw = Swerve.gyro.getYaw();
            a = a+1;
        }

        else if (a==1){
            Launcher.launch(true);
             Launcher.test(false, false, false,false,0,true,false,true);
         Intake.test(false,false, true,false, false,false,0,0,true,true);
          if(Map.leftLauncher.getSelectedSensorVelocity()>19560){
                     
             Timer.delay(.4);
                 Map.intakeLeft.set(ControlMode.PercentOutput,1); 
                     Map.intakeRight.set(ControlMode.PercentOutput,-1);           
                    a=a+1;    
            }
          
                
}
}
}