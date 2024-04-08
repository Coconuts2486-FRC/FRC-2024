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
        Map.rightLauncher.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.leftLauncher.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // rightIntake is what the pivot encoder is wired to. (Absolute encoder)
        Map.rightLauncher.setNeutralMode(NeutralMode.Coast);
        Map.leftLauncher.setNeutralMode(NeutralMode.Coast);
        pivot.setNeutralMode(NeutralMode.Brake);
        Map.leftLauncher.setInverted(true);
         Map.rightLauncher.setInverted(true);
        goTo45 = false;
        angleTuner = 0;

        // Map.rightLauncher.config_kP(0,1);
        // Map.leftLauncher.config_kP(0,1);
        // Map.rightLauncher.config_kI(0,.1);
        // Map.leftLauncher.config_kI(0,.1);

        Map.leftLauncher.configOpenloopRamp(0.1);
        Map.rightLauncher.configOpenloopRamp(0.1);

        Map.rightLauncher.config_kF(0, 0.1);
        Map.leftLauncher.config_kF(0, 0.1);
        Map.rightLauncher.config_kP(0, 0.9);
        Map.leftLauncher.config_kP(0, 0.9);
        Map.rightLauncher.config_kI(0, 0.0055);
        Map.leftLauncher.config_kI(0, 0.0055);
          
        Map.rightLauncher.config_IntegralZone(0, 300);
        Map.leftLauncher.config_IntegralZone(0, 300);

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
            angleTuner = angleTuner - 0.5;
            Timer.delay(.09);
        }

        // Bottom half of the D-Pad, decrease angle
        else if (POV == 180 || POV == 255 || POV == 135) {
            angleTuner = angleTuner + 0.5;
            Timer.delay(.09);
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
        double regressionSlope = -0.27825;
      //  double regressionIntercept = 64.6915;
      double regressionIntercept = 69;

        // Check for "Bad Distance" from the PiVision
        if (distanceFromSpeaker == -999) {
            return 45;
        } else {
            return regressionSlope * distanceFromSpeaker + regressionIntercept;
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
    public static void run(boolean fortyFive, boolean sixty, double tuner, boolean autoFortyFive, boolean regression, boolean kill) {
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
        } else if (regression) {
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
            pivot.set(ControlMode.PercentOutput, pivotPidUP.calculate(pivotEncoder.getAbsolutePosition(), 45 + tuner));
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

    public static void launch(boolean button) {

        if (button) {
            Map.leftLauncher.set(ControlMode.Velocity, 18000);
            Map.rightLauncher.set(ControlMode.Velocity, -18000);
            SmartDashboard.putNumber("rightLaunch", Map.rightLauncher.getSelectedSensorVelocity());
            SmartDashboard.putNumber("leftLaunch", Map.leftLauncher.getSelectedSensorVelocity());
        } else {
             SmartDashboard.putNumber("rightLaunch", Map.rightLauncher.getSelectedSensorVelocity());
            SmartDashboard.putNumber("leftLaunch", Map.leftLauncher.getSelectedSensorVelocity());
            Map.leftLauncher.set(ControlMode.PercentOutput, 0);
            Map.rightLauncher.set(ControlMode.PercentOutput, 0);
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
    Map.leftLauncher.set(ControlMode.Velocity, 16000);
    Map.rightLauncher.set(ControlMode.Velocity, 16000);
    SmartDashboard.putNumber("rightLaunch",
    Map.rightLauncher.getSelectedSensorVelocity());
    SmartDashboard.putNumber("leftLaunch",
    Map.leftLauncher.getSelectedSensorVelocity());
    } else {
    Map.leftLauncher.set(ControlMode.PercentOutput, 0);
    Map.rightLauncher.set(ControlMode.PercentOutput, 0);
    }
    }

    public static double distanceFrom45() {
        return Math.abs(
                pivotEncoder.getAbsolutePosition() - (45 + angleTuner));
    }
}