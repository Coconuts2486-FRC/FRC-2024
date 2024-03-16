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
        }

        // Step #1: HHH
        else if (a == 1) {
            Intake.run(false, false, true, false, false, false, 0, 0, false, red);
            Launcher.launch(true);
            if (Map.leftLauncher.getSelectedSensorVelocity()>16000){
                  Timer.delay(.3);
                a=2;
            }
       
        }

        // Step #2: HHH
        else if (a == 2) {
            Intake.run(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, -.3, 0, false);
            Timer.delay(1);
            a=3;
        } 
        
        // Step #3: HHH
        else if (a == 3) {
                Intake.run(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, 0, false);
        } 
        
        // Step #4: HHH
        else if (a == 4) {
           
        }

        // Step #5: HHH
        else if (a == 5) {
           
        }
    }

}
