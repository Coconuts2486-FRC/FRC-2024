package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;
import frc.robot.Launcher;
import frc.robot.Map;
import frc.robot.Swerve;
import frc.robot.RaspberryPi;

/**
 * AutoPaths Control Class *
 */
@SuppressWarnings("removal")
public class Auto{

    public static int a = 0;
    public static double autoYaw;
    public static double startTime;
    public static PIDController twistPid = new PIDController(.006, 0.0, 0);

    // Change names later
    public static void init() {
        startTime = 0;
        a = 0;

    }

    /**
     * AUTO: 2-piece; straight motion from spaker
     * 
     * @param red Are we red alliance?
     */
    public static void twoPieceStraightFromSpeaker(boolean red) {

        // Step #0: HHH
        if (a == 0) {
           // Intake.run(false, false, false, false, false, false, 0, 0, false, red);
            // Launcher.run(false, false, false, false, 0, red, false, true,0);
            Map.swerve.realignToField(true);
            autoYaw = Swerve.gyro.getYaw();
            a = 1;
            Swerve.reZeroPosition();
            Map.odometry.init();
        }

        // Step #1: HHH
        else if (a == 1) {
            Intake.run(false, false, true, false, false, false, 0, 0, false, red);
            Launcher.launchAuto(true);
            if (Map.leftLauncher.getSelectedSensorVelocity()>16000){
                  Timer.delay(.3);
                a=2;
            }
       
        }

        // Step #2: HHH
        else if (a == 2) {
            Intake.run(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launchAuto(false);
            Map.swerve.drive(0, -.3, 0, false);
            if(Map.odometry.calculatePosition()[1]>=4000000){
                     Map.swerve.drive(0, 0, 0, false);
                a=3;
            }
            
        } 
        
        // Step #3: HHH
        else if (a == 3) {
                Intake.run(true, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launchAuto(false);
            RaspberryPi.targetGamePiece(true,false);
            if(Map.lightStop.get()){
                Map.swerve.drive(0, 0, 0, false);
                a = 4;
            }
        } 
        
        // Step #4: HHH
        else if (a == 4) {
              Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launchAuto(false);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, -0,
            Misc.getSelectedColor()), false);
            RaspberryPi.targetGamePiece(false,false);
            if(red){
                if(Math.abs(RaspberryPi.getTagX4())<4){
                   Map.swerve.drive(0, 0, 0, false);
                    a = 5;
                }
            }else{
                 if(Math.abs(RaspberryPi.getTagX7())<4){
                   Map.swerve.drive(0, 0, 0, false);
                    a = 5;
            }
    

        }
    }
        // Step #5: HHH
        else if (a == 5) {
             Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launchAuto(false);
            RaspberryPi.targetGamePiece(false,false);
             Map.swerve.drive(0, 0, 0, false);
             if(Math.abs(Launcher.pivotEncoder.getAbsolutePosition()-Launcher.regressionForAngle(red))<.3){
                a=6;
             }
           
        }else if (a == 6) {
              Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launchAuto(true);
            RaspberryPi.targetGamePiece(false,false);
             Map.swerve.drive(0, 0, 0, false);
             if(Map.leftLauncher.getSelectedSensorVelocity()>16300){
                a = 7;
             }

        }else if (a == 7){
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launchAuto(false);
            RaspberryPi.targetGamePiece(false,false);
             Map.swerve.drive(0, 0, 0, false);
        }
    }

}
