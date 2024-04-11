package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("removal")
public class Launcher {

    static boolean goTo45 = false;
    public static double angleTuner = 0;

    public static TalonFX pivot = new TalonFX(17);

    // encoder offset is 26
    public static CANCoder pivotEncoder = new CANCoder(31);

    public static PIDController pivotPidDown = new PIDController(.045, 0.0000, 0.00005);
    public static PIDController pivotPidUP = new PIDController(.045, 0.0000, 0.000);
    public static PIDController regressionPidUp = new PIDController(.0516, 0, 0);
    public static PIDController regressionPidDown = new PIDController(.057, 0, 0.0001);

    /**
     * Initializes the launcher by setting the selected feedback sensor.
     */
    public static void init() {
       // pivotPidUP.enableContinuousInput(0, 360);
        pivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.topLauncher.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.bottomLauncher.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // rightIntake is what the pivot encoder is wired to. (Absolute encoder)
        Map.topLauncher.setNeutralMode(NeutralMode.Coast);
        Map.bottomLauncher.setNeutralMode(NeutralMode.Coast);
        pivot.setNeutralMode(NeutralMode.Brake);
        Map.bottomLauncher.setInverted(true);
         Map.topLauncher.setInverted(true);
        goTo45 = false;
        angleTuner = 0;

        // Map.rightLauncher.config_kP(0,1);
        // Map.leftLauncher.config_kP(0,1);
        // Map.rightLauncher.config_kI(0,.1);
        // Map.leftLauncher.config_kI(0,.1);

        Map.bottomLauncher.configOpenloopRamp(0.1);
        Map.topLauncher.configOpenloopRamp(0.1);

        Map.topLauncher.config_kF(0, 0.1);
        Map.bottomLauncher.config_kF(0, 0.1);
        Map.topLauncher.config_kP(0, 0.9);
        Map.bottomLauncher.config_kP(0, 0.9);
        Map.topLauncher.config_kI(0, 0.0055);
        Map.bottomLauncher.config_kI(0, 0.0055);
          
        Map.topLauncher.config_IntegralZone(0, 300);
        Map.bottomLauncher.config_IntegralZone(0, 300);

    }

    /**
     * Disable the Launcher with a button press
     * 
     * @param button The disable button
     */
    public static void disable(boolean button) {
        goTo45 = false;
        if (button) {
            pivot.setNeutralMode(NeutralMode.Coast);
        } else {
            pivot.setNeutralMode(NeutralMode.Brake);
        }
    }

    /**
     * Manually tune the angle of the Launcher (in degrees)
     * 
     * Each press of the D-Pad increases/decreases the angle by 0.5º
     * 
     * @param POV D-Pad of a controller (which is being pressed?)
     * @return angular adjustment (in degrees)
     */

    public static double manualAngleTuner(int POV) {

        // Top half of the D-Pad, increase angle
        if (POV == 0 || POV == 315 || POV == 45) {
            angleTuner = angleTuner - 0.1;
           
        }

        // Bottom half of the D-Pad, decrease angle
        else if (POV == 180 || POV == 255 || POV == 135) {
            angleTuner = angleTuner + 0.1;
           
        }

        // Either right or left of D-Pad, zero out the angle
        else if (POV == 90 || POV == 270) {
            angleTuner = 0;
        }

        // Maximum "tuned" angle is ±10º
        // angleTuner = Math.min(Math.max(angleTuner, -10.), 10.);
        SmartDashboard.putNumber("angle Tuner", angleTuner);
        return angleTuner;
    }

    /**
     * Compute the Launcher angle from the distance regression
     * 
     * @param red Are we red alliance?
     * @return The angle at which to place the Launcher
     */
    public static double regressionForAngle(boolean red) {
        double distanceFromSpeaker = RaspberryPi.getSpeakerCenterZ(red);
        double a = -6.6E-6;
        double b = 3.65E-3;
        double c = -.706;
        
      //  double regressionIntercept = 64.6915;
      double regressionIntercept = 75.6;

        // Check for "Bad Distance" from the PiVision
        if (distanceFromSpeaker == -999) {
            return 45;
        } else {
            return (a*Math.pow(distanceFromSpeaker, 3)) + (b*Math.pow(distanceFromSpeaker,2)) + (c*distanceFromSpeaker)+(regressionIntercept);
        }
    }

    /**
     * The main RUN algorithm
     * 
     * @param fortyFive     boolean, go to 45º
     * @param sixty         boolean, go to 60º
     * @param tuner         double, tuned value, presumably from the tuning function
     * @param autoFortyFive boolean, auto 45º -- how is this different from
     *                      fortyFive?
     */
    public static void run(boolean fortyFive, boolean sixty, double tuner, boolean autoFortyFive, boolean regression, boolean kill, boolean targetRegress) {
        if(kill){
              goTo45 = false;
              sixty = false;
              pivot.set(ControlMode.PercentOutput, 0);
            
        }
        else if (autoFortyFive) {
            goTo45 = true;
        }

        else if (fortyFive) {
            goTo45 = !goTo45;
        }

        if (sixty) {
            pivot.set(ControlMode.PercentOutput, pivotPidDown.calculate(pivotEncoder.getAbsolutePosition(), 60));
        } 
        else if (targetRegress){
          if(Math.abs(RaspberryPi.getSpeakerCenterX(Misc.getSelectedColor())) < 7)  {
         if (regressionForAngle(Misc.getSelectedColor()) < 46) {
                pivot.set(ControlMode.PercentOutput, regressionPidUp.calculate(pivotEncoder.getAbsolutePosition(),
                        (regressionForAngle(Misc.getSelectedColor()) - 1) + tuner));
            } else {
                pivot.set(ControlMode.PercentOutput, regressionPidDown.calculate(pivotEncoder.getAbsolutePosition(),
                        (regressionForAngle(Misc.getSelectedColor()) + .3) + tuner));
            }

        
        }else{
              pivot.set(ControlMode.PercentOutput, pivotPidUP.calculate(pivotEncoder.getAbsolutePosition(), 45 + tuner));
        }
    }
        else if (regression) {
            if (regressionForAngle(Misc.getSelectedColor()) < 46) {
                pivot.set(ControlMode.PercentOutput, regressionPidUp.calculate(pivotEncoder.getAbsolutePosition(),
                        (regressionForAngle(Misc.getSelectedColor()) - 1) + tuner));
            } else {
                pivot.set(ControlMode.PercentOutput, regressionPidDown.calculate(pivotEncoder.getAbsolutePosition(),
                        (regressionForAngle(Misc.getSelectedColor()) + .3) + tuner));
            }

        } else if(Map.driver.getRawButton(3)) {
            pivot.set(ControlMode.PercentOutput, 0);
            if(pivotEncoder.getAbsolutePosition()<4){
                goTo45 = false;
            }

        }else if (goTo45) {
            pivot.set(ControlMode.PercentOutput, pivotPidUP.calculate(pivotEncoder.getAbsolutePosition(), 46.5 + tuner));
        } else {
            pivot.set(ControlMode.PercentOutput, 0);
        }

    }

    /**
     * Launch!
     * 
     * @param button "The Button"
     */

    // public static void testIntake (boolean button){
    // if(button){
    // Map.leftIntake.set(ControlMode.PercentOutput,1);
    // Map.rightIntake.set(ControlMode.PercentOutput,1);
    // }else{
    // Map.leftIntake.set(ControlMode.PercentOutput,0);
    // Map.rightIntake.set(ControlMode.PercentOutput,0);
    // }
    // }

    public static void launch(boolean button,boolean kateLaunch) {

        if (button) {
            Map.bottomLauncher.set(ControlMode.Velocity, 10000);
            Map.topLauncher.set(ControlMode.Velocity, 16000);
            SmartDashboard.putNumber("rightLaunch", Map.topLauncher.getSelectedSensorVelocity());
            SmartDashboard.putNumber("leftLaunch", Map.bottomLauncher.getSelectedSensorVelocity());
        } else if (kateLaunch){
   Map.bottomLauncher.set(ControlMode.Velocity, 10000);
            Map.topLauncher.set(ControlMode.Velocity, 10000);
            SmartDashboard.putNumber("rightLaunch", Map.topLauncher.getSelectedSensorVelocity());
            SmartDashboard.putNumber("leftLaunch", Map.bottomLauncher.getSelectedSensorVelocity());
            if (Map.topLauncher.getSelectedSensorVelocity()>10000){
                Map.leftIntake.set(ControlMode.PercentOutput, 1 );
                Map.rightIntake.set(ControlMode.PercentOutput, 1);                                

            }
            else if (Map.topLauncher.getSelectedSensorVelocity()<8000){
                 Map.leftIntake.set(ControlMode.PercentOutput, 0 );
                Map.rightIntake.set(ControlMode.PercentOutput, 0);        
            }
        }else {
             SmartDashboard.putNumber("rightLaunch", Map.topLauncher.getSelectedSensorVelocity());
            SmartDashboard.putNumber("leftLaunch", Map.bottomLauncher.getSelectedSensorVelocity());

            Map.bottomLauncher.set(ControlMode.PercentOutput, 0);
            Map.topLauncher.set(ControlMode.PercentOutput, 0);
        }
    }

    
    /**
     * AutoLaunch! (The "Doomsday Device")
     * 
     * How is this different functionally from ``launch``?
     * 
     * @param button "The Button"
     */
    public static void launchAuto(boolean button) {
    if (button) {
    Map.bottomLauncher.set(ControlMode.Velocity, 16000);
    Map.topLauncher.set(ControlMode.Velocity, 16000);
    SmartDashboard.putNumber("rightLaunch",
    Map.topLauncher.getSelectedSensorVelocity());
    SmartDashboard.putNumber("leftLaunch",
    Map.bottomLauncher.getSelectedSensorVelocity());
    } else {
    Map.bottomLauncher.set(ControlMode.PercentOutput, 0);
    Map.topLauncher.set(ControlMode.PercentOutput, 0);
    }
    }

    public static double distanceFrom45() {
        return Math.abs(
                pivotEncoder.getAbsolutePosition() - (45 + angleTuner));
    }
}