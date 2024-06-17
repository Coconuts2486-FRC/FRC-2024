package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher {

    static boolean goTo45 = false;
    public static double angleTuner = 0;

    public static TalonFX pivot = new TalonFX(17);

    // encoder offset is 26
    public static CANcoder pivotEncoder = new CANcoder(31);

    public static OpenLoopRampsConfigs ramp = new OpenLoopRampsConfigs();
    
    public static Slot0Configs shotPIDConfigs = new Slot0Configs();

    public static PIDController pivotPidDown = new PIDController(.045, 0.0000, 0.00005);
    public static PIDController pivotPidUP = new PIDController(.056, 0.0000, 0.000001);
    public static PIDController regressionPidUp = new PIDController(.0546, 0, 0);
    public static PIDController regressionPidDown = new PIDController(.057, 0, 0.0001);

    /**
     * Initializes the launcher by setting the selected feedback sensor.
     */
    public static double encoderAsDegrees(){
        return pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }
    public static void init() {
      
       // pivotPidUP.enableContinuousInput(0, 360);
        // rightIntake is what the pivot encoder is wired to. (Absolute encoder)
        Map.topLauncher.getConfigurator().apply(Map.coast);
        Map.bottomLauncher.getConfigurator().apply(Map.coast);
        pivot.getConfigurator().apply(Map.brake);
        Map.bottomLauncher.setInverted(true);
         Map.topLauncher.setInverted(true);
        goTo45 = false;
        angleTuner = 0;

        shotPIDConfigs.kV = .1;
        shotPIDConfigs.kP = 0.9;
        shotPIDConfigs.kI = 0.0055;

        Map.bottomLauncher.getConfigurator().apply(shotPIDConfigs);
        Map.topLauncher.getConfigurator().apply(shotPIDConfigs);


    }

    /**
     * Disable the Launcher with a button press
     * 
     * @param button The disable button
     */
    public static void disable(boolean button) {
        goTo45 = false;
        if (button) {
            pivot.getConfigurator().apply(Map.coast);
        } else {
            pivot.getConfigurator().apply(Map.brake);
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
      double regressionIntercept = 79.;
      //75.6

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
              pivot.setControl(new DutyCycleOut(0));
            
        }
        else if (autoFortyFive) {
            goTo45 = true;
        }

        else if (fortyFive) {
            goTo45 = !goTo45;
        }

        if (sixty) {
            pivot.setControl(new DutyCycleOut(pivotPidDown.calculate(encoderAsDegrees(), 61)));
        } 
        else if (targetRegress){
          if(Math.abs(RaspberryPi.getSpeakerCenterX(Misc.getSelectedColor())) < 7)  {
         if (regressionForAngle(Misc.getSelectedColor()) < 46) {
                pivot.setControl(new DutyCycleOut(regressionPidUp.calculate(encoderAsDegrees(),(regressionForAngle(Misc.getSelectedColor()) - 1) + tuner)));
            } else {
                pivot.setControl(new DutyCycleOut(regressionPidDown.calculate(encoderAsDegrees(),(regressionForAngle(Misc.getSelectedColor()) + .3) + tuner)));
            }

        
        }else{
              pivot.setControl(new DutyCycleOut(pivotPidUP.calculate(encoderAsDegrees(), 45 + tuner)));
        }
    }
        else if (regression) {
            if (regressionForAngle(Misc.getSelectedColor()) < 46) {
                pivot.setControl(new DutyCycleOut(regressionPidUp.calculate(encoderAsDegrees(),
                        (regressionForAngle(Misc.getSelectedColor()) - 1) + tuner)));
            } else {
                pivot.setControl(new DutyCycleOut(regressionPidDown.calculate(encoderAsDegrees(),
                        (regressionForAngle(Misc.getSelectedColor()) + .3) + tuner)));
            }

        } else if(Map.driver.getRawButton(3)) {
            pivot.setControl(new DutyCycleOut(0));
            if(encoderAsDegrees()<4){
                goTo45 = false;
            }

        }else if (goTo45) {
            if (Math.abs(Map.elevator.getPosition().getValueAsDouble())>14.65){
                 pivot.setControl(new DutyCycleOut(pivotPidUP.calculate(encoderAsDegrees(), 36 + tuner)));
            }else{
            pivot.setControl(new DutyCycleOut(pivotPidUP.calculate(encoderAsDegrees(), 45 + tuner)));
            }
        } else {
            pivot.setControl(new DutyCycleOut(0));
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

    public static void launch(boolean connorLaunch,boolean kateLaunch) {
        // change velocities for Phoenix v6
        if (connorLaunch) {
            Map.bottomLauncher.setControl(new VelocityDutyCycle(48.8));
            Map.topLauncher.setControl(new VelocityDutyCycle(78.125));
            SmartDashboard.putNumber("rightLaunch", Map.topLauncher.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("leftLaunch", Map.bottomLauncher.getVelocity().getValueAsDouble());
        } else if (kateLaunch){
   Map.bottomLauncher.setControl(new VelocityDutyCycle( 48.8));
            Map.topLauncher.setControl(new VelocityDutyCycle (48.8));
            SmartDashboard.putNumber("rightLaunch", Map.topLauncher.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("leftLaunch", Map.bottomLauncher.getVelocity().getValueAsDouble());
            if (Map.topLauncher.getVelocity().getValueAsDouble()>48.8){
                Map.intakeBottom.set(ControlMode.PercentOutput, 1);
                Map.intakeTop.set(ControlMode.PercentOutput, 1);                                

            }
            else if (Map.topLauncher.getVelocity().getValueAsDouble()<39.06){
                 Map.intakeBottom.set(ControlMode.PercentOutput, 0);
                Map.intakeTop.set(ControlMode.PercentOutput, 0);        
            }
        }else {
             SmartDashboard.putNumber("rightLaunch", Map.topLauncher.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("leftLaunch", Map.bottomLauncher.getVelocity().getValueAsDouble());

            Map.bottomLauncher.setControl(new DutyCycleOut(0));
            Map.topLauncher.setControl(new DutyCycleOut(0));
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
        // has to change for v6
    Map.bottomLauncher.setControl(new VelocityDutyCycle (8));
    Map.topLauncher.setControl(new VelocityDutyCycle (8));
    SmartDashboard.putNumber("rightLaunch",
    Map.topLauncher.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("leftLaunch",
    Map.bottomLauncher.getVelocity().getValueAsDouble());
    } else {
    Map.bottomLauncher.setControl(new DutyCycleOut(0));
    Map.topLauncher.setControl(new DutyCycleOut(0));
    }
    }

    public static double distanceFrom45() {
        return Math.abs(
                encoderAsDegrees() - (45 + angleTuner));
    }
}