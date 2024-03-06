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
    private static double robotToTagZ;
    private static final int positionIntake = 2;
    private static final int positionShoot = 3;
    private static final int positionZero = 1;

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

    // /**
    // * Spins the intake wheels based on an axis input and the extension state of
    // the
    // * intake mechanism.
    // *
    // * @param axis The axis input to control the speed of the intake wheels.
    // * @param extended The state of the intake mechanism (extended or retracted).
    // */
    // // NOTE: The docstring does not match the function arguments!!!
    // public static void intakeSpin() {

    // if (Map.movementIntake.getSelectedSensorPosition() > 80000) {
    // Map.intakeRight.set(ControlMode.PercentOutput, -.32);
    // Map.intakeLeft.set(ControlMode.PercentOutput, .32);
    // if (Map.lightStop.get()) {
    // Map.intakeRight.set(ControlMode.PercentOutput, 0);
    // Map.intakeLeft.set(ControlMode.PercentOutput, 0);
    // }
    // } else {
    // Map.intakeRight.set(ControlMode.PercentOutput, .0);
    // Map.intakeLeft.set(ControlMode.PercentOutput, .0);
    // }
    // }

    /**
     * This is the RUN function that is being used!
     * 
     * NOTE: Add more description here about what this function does
     * 
     * @param intakePositionButton  "Make the Intake Go Out" button press
     *                              (press-and-hold)
     * @param scoringPositionToggle "Move intake out for proper shooter feed" button
     *                              toggle
     * @param launchNote            Launch the note by feeding the shooter button
     *                              (press-and-hold)
     * @param reZeroIntake          Re-zero the intake button (press-and-hold)
     * @param targetButton          Button that checks the camera input for how
     *                              close to wall we are (pull in intake)
     * @param autoScoreTrue         Holds the intake at a specified position for
     *                              AUTO
     * @param intakeAxis            "Axis Thing" on controller that causes the
     *                              rollers to intake (with speed control)
     * @param outtakeAxis           "Axis Thing" on controller that causes the
     *                              rollers to outtake (with speed control)
     * @param autoZero              Holds the intake at the zero position for AUTO
     * @param red                   Are we red alliance?
     */
    public static void run(boolean intakePositionButton, boolean scoringPositionToggle,
            boolean launchNote, boolean reZeroIntake, boolean targetButton, boolean autoScoreTrue,
            double intakeAxis, double outtakeAxis, boolean autoZero, boolean red) {
        // put number to smart dashboard
        SmartDashboard.putNumber("intakeZone", Launcher.distanceFrom45());
        // toggle for scoring position
        // trippleToggle = 1;
        // toggleScore = false;
        // toggleOut = false;
        if (targetButton) {
            // Select which AprilTag based on alliance selection
            if (red) {
                robotToTagZ = RaspberryPi.getTagZ4();
            } else {
                robotToTagZ = RaspberryPi.getTagZ7();
            }
            // `toggleScore` means place the intake in the "shooting position"
            if (robotToTagZ > 50 && robotToTagZ != -999) {
                toggleScore = true;
            } else {
                toggleScore = false;
            }
        }

        // NOTE: I don't do this anymore
        // if (launchNote) {
        // shotClock = Timer.getFPGATimestamp();
        // }

        // Check status of scoring toggles
        if (autoScoreTrue) {
            toggleScore = true;
        } else if (scoringPositionToggle) {
            toggleScore = !toggleScore;
        }

        // If button is pressed -- I don't use this anymore
        // if (intakePositionButton) {
        // toggleOut = !toggleOut;
        // if (Launcher.distanceFrom45() > 2.) {
        // toggleOut = false;
        // }
        // }

        // If "make it go out" button is pressed, and the pivot (Launcher) is in
        // the correct position, and the beam-break is NOT triggered, set the
        // tripple toggle to positionIntake
        if (intakePositionButton == true && Launcher.distanceFrom45() < 2.0 && Map.lightStop.get() == false) {
            trippleToggle = positionIntake;
        }
        // If AUTO hold-zero OR the pivot is steeper than 50º
        else if (autoZero ||
                Launcher.pivotEncoderAngle(Map.intakeRight.getSelectedSensorPosition()) > 50.){
            trippleToggle = positionZero;
        }
        // If we WANT to be in scoring position and the intake is NOT too far in (ticks
        // are backwards), set triple toggle to positionShoot
        else if (toggleScore == true && Map.intakeRight.getSelectedSensorPosition() <= 1300) {
            trippleToggle = positionShoot;
        }
        // Default fail-safe is zero position
        else {
            trippleToggle = positionZero;
        }

        // This is where we actually do stuff!!!å

        // Move to intake position & turn on the intake rollers (intakeLeft & intakeRight)
        if (trippleToggle == positionIntake) {
            // Map.movementIntake.set(ControlMode.PercentOutput,intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(),98000));
            Map.movementIntake.set(ControlMode.Position, 98000);
            Map.intakeLeft.set(ControlMode.PercentOutput, .39);
            Map.intakeRight.set(ControlMode.PercentOutput, .39);
        }

        // If intaking (present tense), and the rezero button is pressed, then rezero
        else if (reZeroIntake) {
            Map.movementIntake.set(ControlMode.PercentOutput, -.3);
            // Limit switch at zero position -> HOME the values
            if (Map.intakeStop.get()) {
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
                Map.movementIntake.setSelectedSensorPosition(0);
            }
        }

        // Move to general shooting position and enable the ability to shoot
        else if (trippleToggle == positionShoot) {
            // Map.movementIntake.set(ControlMode.PercentOutput,
            // intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 37000));
            Map.movementIntake.set(ControlMode.Position, 37000);

            // Map.movementIntake.set(ControlMode.Position, 34500);
            // If the elevator is UP, we are scoring in the AMP, so we OUTTAKE the
            //   gamepiece rather than feed it into the shooter mechanism
            if (Elevator.toggleScore && Map.leftElevator.getSelectedSensorPosition() < -80000) {
                Map.intakeLeft.set(ControlMode.PercentOutput, -1);
                Map.intakeRight.set(ControlMode.PercentOutput, -1);
            }
            // If the shooter wheels are up to speed, then automatically insert
            //  the note into the shooter if the `launchNote` button is pressed
            //  NOTE: At present, the `launchNote` button is the SAME as the Launcher.launch button
            else if (launchNote && Map.leftLauncher.getSelectedSensorVelocity() > 20500) {
                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                Map.intakeRight.set(ControlMode.PercentOutput, 1);
            }
            // If the shooter wheels are NOT up to speed, manually control the intake/outtake
            else if (Map.leftLauncher.getSelectedSensorVelocity() < 19500) {
                Map.intakeLeft.set(ControlMode.PercentOutput, (intakeAxis - outtakeAxis));
                Map.intakeRight.set(ControlMode.PercentOutput, -(outtakeAxis - intakeAxis) * 1.4);
            }
            // Other cases, we just sit on our hands
            else {}
        } 
        
        // Move the intake to the zero position
        else if (trippleToggle == positionZero) {

            // Map.movementIntake.set(ControlMode.PercentOutput,
            // intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 0));
            // Move the intake to zero position
            Map.movementIntake.set(ControlMode.Position, 0);

            // Re-zero intake if limit switch is pressed
            if (Map.intakeStop.get()) {
                Map.movementIntake.setSelectedSensorPosition(0);
            }

            // If the elevator is UP, we are scoring in the AMP, so we OUTTAKE the
            //   gamepiece rather than feed it into the shooter mechanism
            if (Elevator.toggleScore && Map.leftElevator.getSelectedSensorPosition() < -80000) {
                Map.intakeLeft.set(ControlMode.PercentOutput, -1);
                Map.intakeRight.set(ControlMode.PercentOutput, -1);
            }
            // If the shooter wheels are up to speed, then automatically insert
            //  the note into the shooter if the `launchNote` button is pressed
            //  NOTE: At present, the `launchNote` button is the SAME as the Launcher.launch button
            else if (launchNote && Map.leftLauncher.getSelectedSensorVelocity() > 20500) {
                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                Map.intakeRight.set(ControlMode.PercentOutput, 1);
            }
            // If the shooter wheels are NOT up to speed, manually control the intake/outtake
            else if (Map.leftLauncher.getSelectedSensorVelocity() < 19500) {
                Map.intakeLeft.set(ControlMode.PercentOutput, (intakeAxis - outtakeAxis));
                Map.intakeRight.set(ControlMode.PercentOutput, -(outtakeAxis - intakeAxis) * 1.4);
            }
            // Other cases, we just sit on our hands
            else {}
        }
    }

    // NOTE: The following functions were subsumed into other functions

    // /**
    // * Auto Spin (you spin me right round, baby, right round)
    // *
    // * @param spin like a record
    // */
    // public static void autoSpin(boolean spin) {
    // if (spin) {
    // Map.intakeLeft.set(ControlMode.PercentOutput, -1);
    // Map.intakeRight.set(ControlMode.PercentOutput, -1);

    // } else if (spin = false) {
    // Map.intakeLeft.set(ControlMode.PercentOutput, 0);
    // Map.intakeRight.set(ControlMode.PercentOutput, 0);

    // }
    // }

    // /**
    // * Scores the intake mechanism based on a button input.
    // *
    // * @param button The button input to control the scoring of the intake
    // * mechanism.
    // */
    // public static void score(boolean button) {
    // // change this -- Uh, to what?
    // if (Map.rightElevator.getSelectedSensorPosition() < -54000) {
    // if (button) {
    // Map.intakeRight.set(ControlMode.PercentOutput, -.5);
    // Map.intakeLeft.set(ControlMode.PercentOutput, -.5);
    // } else {
    // Map.intakeRight.set(ControlMode.PercentOutput, 0);
    // Map.intakeLeft.set(ControlMode.PercentOutput, 0);
    // }
    // } else {
    // if (button) {
    // Map.intakeRight.set(ControlMode.PercentOutput, -1);
    // Map.intakeLeft.set(ControlMode.PercentOutput, -1);
    // } else {
    // Map.intakeRight.set(ControlMode.PercentOutput, 0);
    // Map.intakeLeft.set(ControlMode.PercentOutput, 0);
    // }
    // }
    // }
}
