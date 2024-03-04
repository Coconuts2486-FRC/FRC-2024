package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.RaspberryPi;

/**
 * This class represents an intake mechanism for a robot.
 */
public class Intake {
    public static PIDController intakePID = new PIDController(.00007, 0.00000, 0.00000093);
    private static boolean toggleOut = false;
    private static boolean toggleScore = false;
    private static int trippleToggle = 1;
    public static double shotClock;

    /**
     * Initializes the intake mechanism.
     */
    public static void init() {
        toggleOut = false;
        toggleScore = false;
        Map.movementIntake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        Map.intakeRight.setInverted(false);

        Map.movementIntake.setSelectedSensorPosition(0);
        Map.movementIntake.setNeutralMode(NeutralMode.Brake);
        trippleToggle = 1;
        Map.movementIntake.config_kP(0, 0.55);
        Map.movementIntake.config_kI(0, 0.0000);
        Map.movementIntake.config_kD(0, 0.000001);
        // Map.rightLauncher.config_kP(0,1);
        // Map.leftLauncher.config_kP(0,1);
        // Map.rightLauncher.config_kI(0,.1);
        // Map.leftLauncher.config_kI(0,.1);

        Map.leftLauncher.configOpenloopRamp(0.1);
        Map.rightLauncher.configOpenloopRamp(0.1);

        Map.rightLauncher.config_kF(0, 0.13);
        Map.leftLauncher.config_kF(0, 0.13);
        Map.rightLauncher.config_kP(0, 0.7);
        Map.leftLauncher.config_kP(0, 0.7);
        Map.rightLauncher.config_kI(0, 0.005);
        Map.leftLauncher.config_kI(0, 0.005);
        Map.rightLauncher.config_IntegralZone(0, 300);
        Map.leftLauncher.config_IntegralZone(0, 300);

    }

    /**
     * Disable the Intake with a button press
     * 
     * @param button The disable button
     */
    public static void disable(boolean button) {
        toggleOut = false;
        toggleScore = false;
        trippleToggle = 1;

        if (button) {
            Map.movementIntake.setNeutralMode(NeutralMode.Coast);
        } else {
            Map.movementIntake.setNeutralMode(NeutralMode.Brake);
        }
    }

    /**
     * Runs the intake mechanism based on a button input.
     * 
     * @param button The button input to control the intake mechanism.
     */
    // NOTE: There's no code here with this docstring!!!

    /**
     * Spins the intake wheels based on an axis input and the extension state of the
     * intake mechanism.
     * 
     * @param axis     The axis input to control the speed of the intake wheels.
     * @param extended The state of the intake mechanism (extended or retracted).
     */
    // NOTE: The docstring does not match the function arguments!!!
    public static void intakeSpin() {

        if (Map.movementIntake.getSelectedSensorPosition() > 80000) {
            Map.intakeRight.set(ControlMode.PercentOutput, -.32);
            Map.intakeLeft.set(ControlMode.PercentOutput, .32);
            if (Map.lightStop.get()) {
                Map.intakeRight.set(ControlMode.PercentOutput, 0);
                Map.intakeLeft.set(ControlMode.PercentOutput, 0);
            }
        } else {
            Map.intakeRight.set(ControlMode.PercentOutput, .0);
            Map.intakeLeft.set(ControlMode.PercentOutput, .0);
        }
    }

    /**
     * This is the RUN function that is being used!
     * 
     * NOTE: Add more description here about what this function does
     * 
     * @param intakePositionButton  Intake position button press
     * @param scoringPositionToggle Move to scoring position button press
     * @param launchNote            Launch the note button press
     * @param reZeroIntake          Re-zero the intake button press
     * @param targetButton          Target the something button pres
     * @param autoScoreTrue         ???
     * @param intakeAxis            ???
     * @param outtakeAxis           ???
     * @param autoZero              ???
     * @param red                   Are we red alliance?
     */
    public static void run(boolean intakePositionButton, boolean scoringPositionToggle, boolean launchNote,
            boolean reZeroIntake, boolean targetButton, boolean autoScoreTrue, double intakeAxis, double outtakeAxis,
            boolean autoZero, boolean red) {
        // put number to smart dashboard
        SmartDashboard.putNumber("intakeZone",
                Math.abs(Math.abs(Map.intakeRight.getSelectedSensorPosition()) + (-1240 + Launcher.angleTuner)));
        // toggle for scoring position
        // trippleToggle = 1;
        // toggleScore = false;
        // toggleOut = false;
        if (targetButton) {
            if (red) {
                if (RaspberryPi.getTagZ4() > 50 && RaspberryPi.getTagZ4() != -999) {
                    toggleScore = true;
                } else {
                    toggleScore = false;
                }

            } else {
                if (RaspberryPi.getTagZ7() > 50 && RaspberryPi.getTagZ7() != -999) {
                    toggleScore = true;
                } else {
                    toggleScore = false;
                }

            }

        }
        if (launchNote) {
            shotClock = Timer.getFPGATimestamp();
        }

        if (autoScoreTrue) {
            toggleScore = true;
        } else if (scoringPositionToggle) {
            toggleScore = !toggleScore;
        }
        if (intakePositionButton) {
            toggleOut = !toggleOut;
            if (Math.abs(Math.abs(Map.intakeRight.getSelectedSensorPosition()) + (-1240 + Launcher.angleTuner)) > 18) {
                toggleOut = false;
            }
        }
        // if button, and correct position, and light sensor, set the tripple toggle to
        // 2
        if (intakePositionButton == true
                && Math.abs(Math.abs(Map.intakeRight.getSelectedSensorPosition()) + (-1240 + Launcher.angleTuner)) < 18
                && Map.lightStop.get() == false) {
            trippleToggle = 2;
            // if any are false, set it to 3 or 1 depending on if toggle score is true or
            // false
        } else if (intakePositionButton == false
                || Math.abs(Math.abs(Map.intakeRight.getSelectedSensorPosition()) + (-1240 + Launcher.angleTuner)) > 18
                || Map.lightStop.get() == true) {
            if (toggleScore == true && Map.intakeRight.getSelectedSensorPosition() < 1300) {
                trippleToggle = 3;

            } else if (autoZero || toggleScore == false || Launcher.launcherPivot.getSelectedSensorPosition() > 1300) {
                trippleToggle = 1;
            }
            Map.movementIntake.set(ControlMode.PercentOutput, 0);
        }
        // if tripple toggle is 2, set intake to intake position and start intake
        if (trippleToggle == 2) {
            // Map.movementIntake.set(ControlMode.PercentOutput,intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(),98000));
            Map.movementIntake.set(ControlMode.Position, 98000);
            Map.intakeLeft.set(ControlMode.PercentOutput, .39);
            Map.intakeRight.set(ControlMode.PercentOutput, .39);
        } else if (reZeroIntake) {
            Map.movementIntake.set(ControlMode.PercentOutput, -.3);
            if (Map.intakeStop.get()) {
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
                Map.movementIntake.setSelectedSensorPosition(0);
            }
        }
        // if tripple toggle is 3, go to general shooting position and enable the
        // ability to shoot
        else if (trippleToggle == 3) {

            // Map.movementIntake.set(ControlMode.PercentOutput,
            // intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 37000));
            Map.movementIntake.set(ControlMode.Position, 37000);

            // Map.movementIntake.set(ControlMode.Position, 34500);
            // if the shoot button is pressed, check if the elevator is above 20000, and
            // shoot out if it is, shoot if it isn't
            if (Elevator.toggleScore && Map.leftElevator.getSelectedSensorPosition() < -80000) {
                Map.intakeLeft.set(ControlMode.PercentOutput, -1);
                Map.intakeRight.set(ControlMode.PercentOutput, -1);
            }

            else if (launchNote && Map.leftLauncher.getSelectedSensorVelocity() > 20500) {

                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                Map.intakeRight.set(ControlMode.PercentOutput, 1);

            } else if (Map.leftLauncher.getSelectedSensorVelocity() < 19500) {

                Map.intakeLeft.set(ControlMode.PercentOutput, (intakeAxis - outtakeAxis));
                Map.intakeRight.set(ControlMode.PercentOutput, -(outtakeAxis - intakeAxis) * 1.4);

            }

            // if toggle is 1, go to zero position
        } else if (trippleToggle == 1) {

            // Map.movementIntake.set(ControlMode.PercentOutput,
            // intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 0));
            Map.movementIntake.set(ControlMode.Position, 0);

            // if intake is in and elevator is up and not moving, it auto outtakes,
            // otherwise, wait until button is pressed, and do the same as toggle 3
            if (Elevator.toggleScore && Map.leftElevator.getSelectedSensorPosition() < -80000) {
                Map.intakeLeft.set(ControlMode.PercentOutput, -1);
                Map.intakeRight.set(ControlMode.PercentOutput, -1);

            } else if (launchNote && Map.leftLauncher.getSelectedSensorVelocity() > 20500) {

                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                Map.intakeRight.set(ControlMode.PercentOutput, 1);

            } else if (Map.leftLauncher.getSelectedSensorVelocity() < 19500) {

                Map.intakeLeft.set(ControlMode.PercentOutput, (intakeAxis - outtakeAxis));
                Map.intakeRight.set(ControlMode.PercentOutput, -(outtakeAxis - intakeAxis) * 1.4);

            }

            // re-zero intake if limit switch is pressed.
            if (Map.intakeStop.get()) {
                Map.movementIntake.setSelectedSensorPosition(0);
            }
        }

    }

    /**
     * Auto Spin (you spin me right round, baby, right round)
     * 
     * @param spin like a record
     */
    public static void autoSpin(boolean spin) {
        if (spin) {
            Map.intakeLeft.set(ControlMode.PercentOutput, -1);
            Map.intakeRight.set(ControlMode.PercentOutput, -1);

        } else if (spin = false) {
            Map.intakeLeft.set(ControlMode.PercentOutput, 0);
            Map.intakeRight.set(ControlMode.PercentOutput, 0);

        }
    }

    /**
     * Scores the intake mechanism based on a button input.
     * 
     * @param button The button input to control the scoring of the intake
     *               mechanism.
     */
    public static void score(boolean button) {
        // change this -- Uh, to what?
        if (Map.rightElevator.getSelectedSensorPosition() < -54000) {
            if (button) {
                Map.intakeRight.set(ControlMode.PercentOutput, -.5);
                Map.intakeLeft.set(ControlMode.PercentOutput, -.5);
            } else {
                Map.intakeRight.set(ControlMode.PercentOutput, 0);
                Map.intakeLeft.set(ControlMode.PercentOutput, 0);
            }
        } else {
            if (button) {
                Map.intakeRight.set(ControlMode.PercentOutput, -1);
                Map.intakeLeft.set(ControlMode.PercentOutput, -1);
            } else {
                Map.intakeRight.set(ControlMode.PercentOutput, 0);
                Map.intakeLeft.set(ControlMode.PercentOutput, 0);
            }
        }
    }
}
