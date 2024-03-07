package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
@SuppressWarnings("removal")
public class LauncherTest {

    static boolean goTo45 = false;
    public static double angleTuner = 0;

    
    public static TalonFX pivot = new TalonFX(17);
    
    //encoder offset is 26
    public static CANCoder encoder = new CANCoder(31);

    public static PIDController pivotPid = new PIDController(.042, 0.0000, 0.00005);
    public static PIDController pivotPid2 = new PIDController(.042, 0.0000, 0.0003);


    //.
    public static double manualAngleTuner(int POV) {
       

        // Top half of the D-Pad, increase angle
         if (POV == 0 || POV == 315 || POV == 45) {
            angleTuner = angleTuner - 0.5;
            Timer.delay(.05);
        }

        // Bottom half of the D-Pad, decrease angle
        else if (POV == 180 || POV == 255 || POV == 135) {
            angleTuner = angleTuner + 0.5;
            Timer.delay(.08);
        }

        // Either right or left of D-Pad, zero out the angle
        else if (POV == 90 || POV == 270) {
            angleTuner = 0;
        }

        // Maximum "tuned" angle is ±10º
        angleTuner = Math.min(Math.max(angleTuner, -10.), 10.);
        SmartDashboard.putNumber("angle Tuner", angleTuner);
        return angleTuner;
    }
    //.
    
   
    /**
     * Manually tune the angle of the Launcher (in degrees)
     * 
     * Each press of the D-Pad increases/decreases the angle by 0.5º
     * 
     * @param POV D-Pad of a controller (which is being pressed?)
     * @return angular adjustment (in degrees)
     */
    public static void run(boolean fortyFive, boolean sixty, double tuner, boolean autoFortyFive) {

        if (autoFortyFive){
            goTo45 = true;
        }
     
       else if (fortyFive) {
            goTo45 = !goTo45;
        }


        if (sixty) {
            pivot.set(ControlMode.PercentOutput, pivotPid.calculate(encoder.getAbsolutePosition(), 60));
            // pivot.set(ControlMode.PercentOutput, -.1);
        } else if (goTo45) {
            pivot.set(ControlMode.PercentOutput, pivotPid2.calculate(encoder.getAbsolutePosition(), 44+tuner));
            // pivot.set(ControlMode.PercentOutput, .1);
        } else {
            pivot.set(ControlMode.PercentOutput, 0);
        }

    }
}