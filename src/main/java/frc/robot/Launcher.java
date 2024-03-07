package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class represents the launcher of the robot.
 * It provides methods to control the launcher.
 */
@SuppressWarnings("removal")
public class Launcher {
    public static TalonFX pivotLauncher = new TalonFX(17);
    static boolean goTo45 = false;
    static boolean launcherReady = false;
    private static double rand;

    // Proportional Integal Derivative == Speed Controller
    // kp == Constant * Tolerance between current value and stepoint
    // ki == Constant * Integral of the tolerance
    // kd == Constant * Derivative of the tolerance
    public static PIDController pivotPid = new PIDController(.0042, 0.0000, 0);

    public static boolean toggleTarget = false;
    public static double angleTuner = 0;

    /**
     * Initializes the launcher by setting the selected feedback sensor.
     */
    public static void init() {
        pivotLauncher.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.rightLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        Map.leftLauncher.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        // rightIntake is what the pivot encoder is wired to. (Absolute encoder)
        Map.rightIntake.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

        pivotLauncher.setNeutralMode(NeutralMode.Brake);
        Map.rightLauncher.setInverted(true);
        goTo45 = false;
        angleTuner = 0;

    }

    /**
     * Disable the Launcher with a button press
     * 
     * @param button The disable button
     */
    public static void disable(boolean button) {
        goTo45 = false;
        if (button) {
            pivotLauncher.setNeutralMode(NeutralMode.Coast);
        } else {
            pivotLauncher.setNeutralMode(NeutralMode.Brake);
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
        // Limit switch for biggest angle (60º); `false` means pressed
        if (Map.pivotStop.get() == false) {
            angleTuner = 0;
        }

        // Top half of the D-Pad, increase angle
        else if (POV == 0 || POV == 315 || POV == 45) {
            angleTuner = angleTuner - 0.5;
            Timer.delay(.05);
        }

        // Bottom half of the D-Pad, decrease angle
        else if (POV == 180 || POV == 255 || POV == 135) {
            angleTuner = angleTuner + 0.5;
            Timer.delay(.05);
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

    /**
     * Calculates the angle of the launcher based on the distance.
     *
     * @param distance The distance to the target.
     * @return The angle of the launcher.
     */
    public static double calculateAngle(double distance) {
        // insert regression alg. Distance converted to Position.
        // reg alg
        double calculation = distance * 123 - distance * 123;

        double conversion = 2651;
        calculation = (calculation * conversion);

        if (calculation < 0) {
            calculation = 0;
        }
        return calculation;
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
        // rand changes to a new random double between 0 and 1 aprox. every 20ms
        rand = Math.random();
        // if rand is greater than .93, print info to terminal.
        // prints info aprox. 7% of times run

        if (rand > .93) {
            System.out.println(Map.rightIntake.getSelectedSensorPosition());
            System.out.println(pivotLauncher.getSupplyCurrent());
        }

        if (autoFortyFive) {
            goTo45 = true;
        } else if (fortyFive) {
            goTo45 = !goTo45;
        }

        // Test code to stop the pivot from going to far -- didn't work
        // if (Map.rightIntake.getSelectedSensorPosition() < 1050) {
        // goTo45 = false;
        // sixty = false;
        // pivotLauncher.set(ControlMode.PercentOutput, 0);
        // }

        // Default "HOME" position, also used for "point-blank" subwoofer shot
        if (sixty) {
            pivotLauncher.set(ControlMode.PercentOutput,
                    pivotPid.calculate(Map.rightIntake.getSelectedSensorPosition(), pivotAngleEncoder(60.)));
            // If pivot at largest angle (limit switch), stop (false => triggered)
            if (Map.pivotStop.get() == false) {
                pivotLauncher.set(ControlMode.PercentOutput, 0);
            }
        }

        // Intake position selected
        else if (goTo45) {

            // Danger Zone: Pivot will rip something off -- STOP EVERYTHING
            if (pivotEncoderAngle(Map.rightIntake.getSelectedSensorPosition()) < 25.0) {
                goTo45 = false;
                pivotLauncher.set(ControlMode.PercentOutput, 0);
            }

            // As the elevator goes UP, the motor ticks become MORE NEGATIVE
            // If the elevator is in the "right position", then rotate the pivot
            // to score in the AMP.
            else if (Map.leftElevator.getSelectedSensorPosition() < -20000) {
                pivotLauncher.set(ControlMode.PercentOutput,
                        // just in case: -36900 is motor tick to go back to.
                        pivotPid.calculate(Map.rightIntake.getSelectedSensorPosition(), pivotAngleEncoder(32.0)));
            }

            // Final something else
            else {
                pivotLauncher.set(ControlMode.PercentOutput,
                        pivotPid.calculate(Map.rightIntake.getSelectedSensorPosition(),
                                pivotAngleEncoder(45. + tuner)));
            }
        }

        // Turn off
        else {
            pivotLauncher.set(ControlMode.PercentOutput, 0);
        }
    }

    /**
     * Launch!
     * 
     * @param button "The Button"
     */
    public static void launch(boolean button) {

        if (button) {
            Map.leftLauncher.set(TalonSRXControlMode.Velocity, 21000);
            Map.rightLauncher.set(TalonSRXControlMode.Velocity, 21000);
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

    /**
     * Convert pivot encoder tick values to pivot angle in degrees
     * 
     * Encoder value of 1420 is when the pivot head is at 60º. As the head
     * to lower angles, the ticks value move to lower values, also. There
     * are 4096 ticks in the complete circle (360º).
     * 
     * By comparison, the 45º position is approximately 1240 ticks. Inserting
     * this value into this function yields 44.18º.
     * 
     * @param ticks Value from the encoder
     * @return Head angle in degrees.
     */
    public static double pivotEncoderAngle(double ticks) {
        return ((ticks - 1420) / 4096) * 360. + 60.;
    }

    /**
     * Convert from pivot angle in degrees to pivot encoder tick values
     * 
     * @param angle
     * @return
     */

    public static int pivotAngleEncoder(double angle) {
        return (int) ((angle - 60.) / 360. * 4096 + 1420);
    }

    public static double distanceFrom45() {
        return Math.abs(Math.abs(
                pivotEncoderAngle(Map.rightIntake.getSelectedSensorPosition())) - (45 + angleTuner));
    }
}