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

    public static PIDController pivotPidDown = new PIDController(.042, 0.0000, 0.00005);
    public static PIDController pivotPidUP = new PIDController(.042, 0.0000, 0.0003);

    /**
     * Initializes the launcher by setting the selected feedback sensor.
     */
    public static void init() {
        pivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.rightLauncher.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.leftLauncher.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // rightIntake is what the pivot encoder is wired to. (Absolute encoder)
        Map.rightLauncher.setNeutralMode(NeutralMode.Coast);
        Map.leftLauncher.setNeutralMode(NeutralMode.Coast);
        pivot.setNeutralMode(NeutralMode.Brake);
        Map.leftLauncher.setInverted(true);
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
        Map.rightLauncher.config_kP(0, 0.7);
        Map.leftLauncher.config_kP(0, 0.7);
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
     * The main RUN algorithm
     * 
     * @param fortyFive     boolean, go to 45º
     * @param sixty         boolean, go to 60º
     * @param tuner         double, tuned value, presumably from the tuning function
     * @param autoFortyFive boolean, auto 45º -- how is this different from
     *                      fortyFive?
     */
    public static void run(boolean fortyFive, boolean sixty, double tuner, boolean autoFortyFive) {

        if (autoFortyFive) {
            goTo45 = true;
        }

        else if (fortyFive) {
            goTo45 = !goTo45;
        }

        if (sixty) {
            pivot.set(ControlMode.PercentOutput, pivotPidDown.calculate(pivotEncoder.getAbsolutePosition(), 60));
        } else if (goTo45) {
            pivot.set(ControlMode.PercentOutput, pivotPidUP.calculate(pivotEncoder.getAbsolutePosition(), 44 + tuner));
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
    //     if(button){
    //         Map.leftIntake.set(ControlMode.PercentOutput,1);
    //     Map.rightIntake.set(ControlMode.PercentOutput,1);
    //     }else{
    //              Map.leftIntake.set(ControlMode.PercentOutput,0);
    //     Map.rightIntake.set(ControlMode.PercentOutput,0);
    //     }
    // }


    public static void launch(boolean button) {

        if (button) {
            Map.leftLauncher.set(ControlMode.Velocity, 15000);
            Map.rightLauncher.set(ControlMode.Velocity, 15000);
            SmartDashboard.putNumber("rightLaunch", Map.rightLauncher.getSelectedSensorVelocity());
            SmartDashboard.putNumber("leftLaunch", Map.leftLauncher.getSelectedSensorVelocity());
        } else {
            Map.leftLauncher.set(ControlMode.PercentOutput, .0);
            Map.rightLauncher.set(ControlMode.PercentOutput, .0);
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
            Map.leftLauncher.set(ControlMode.Velocity, 21500);
            Map.rightLauncher.set(ControlMode.Velocity, 21500);
            SmartDashboard.putNumber("rightLaunch", Map.rightLauncher.getSelectedSensorVelocity());
            SmartDashboard.putNumber("leftLaunch", Map.leftLauncher.getSelectedSensorVelocity());
        } else {
            Map.leftLauncher.set(ControlMode.Velocity, 10000);
            Map.rightLauncher.set(ControlMode.Velocity, 10000);
        }
    }
}