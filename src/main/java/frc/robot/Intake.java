package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
@SuppressWarnings("removal")
public class Intake {

    public static boolean toggleScore = false;
    public static int trippleToggle = 1;

    public static double robotToTagZ;
    public static  int positionIntake = 2;
    public static  int positionShoot = 3;
    public static int positionZero = 1;

    public static void init() {
      //  toggleOut = false;
        toggleScore = false;
        
        Map.intakeExtend.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        Map.rightIntake.setInverted(false);

        Map.intakeExtend.setSelectedSensorPosition(0);
        Map.intakeExtend.setNeutralMode(NeutralMode.Brake);
        trippleToggle = 1;
        Map.intakeExtend.config_kP(0, 0.55);
        Map.intakeExtend.config_kI(0, 0.0000);
        Map.intakeExtend.config_kD(0, 0.000001);
    }
   /**
     * Disable the Intake with a button press
     * 
     * @param button The disable button
     */
    public static void disable(boolean button) {
       // toggleOut = false;
        toggleScore = false;
        trippleToggle = 1;

        if (button) {
            Map.intakeExtend.setNeutralMode(NeutralMode.Coast);
        } else {
            Map.intakeExtend.setNeutralMode(NeutralMode.Brake);
        }
    }


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
        // if (targetButton) {
        //     // Select which AprilTag based on alliance selection
        //     if (red) {
        //         robotToTagZ = RaspberryPi.getTagZ4();
        //     } else {
        //         robotToTagZ = RaspberryPi.getTagZ7();
        //     }
        //     // `toggleScore` means place the intake in the "shooting position"
        //     if (robotToTagZ > 50 && robotToTagZ != -999) {
        //         toggleScore = true;
        //     } else {
        //         toggleScore = false;
        //     }
        // }

        // Check status of scoring toggles
        if (autoScoreTrue) {
            toggleScore = true;
        } else if (scoringPositionToggle) {
            toggleScore = !toggleScore;
        }


        // If "make it go out" button is pressed, and the pivot (Launcher) is in
        // the correct position, and the beam-break is NOT triggered, set the
        // tripple toggle to positionIntake
        if (intakePositionButton == true && Launcher.distanceFrom45() < 4.0 && Map.lightStop.get() == false) {
            trippleToggle = positionIntake;
        }
        // If AUTO hold-zero OR the pivot is steeper than 50º
        else if (autoZero ||
                Launcher.pivotEncoder.getAbsolutePosition() > 50){
            trippleToggle = positionZero;
        }
        // If we WANT to be in scoring position and the intake is NOT too far in (ticks
        // are backwards), set triple toggle to positionShoot
        else if (toggleScore == true && Launcher.pivotEncoder.getAbsolutePosition() <= 50) {
            trippleToggle = positionShoot;
        }
        // Default fail-safe is zero position
        else {
            trippleToggle = positionZero;
        }

        // This is where we actually do stuff!!!å

        // Move to intake position & turn on the intake rollers (leftIntake & rightIntake)
        if (trippleToggle == positionIntake) {
            Map.intakeExtend.set(ControlMode.Position, 98000);
            Map.leftIntake.set(ControlMode.PercentOutput, .39);
            Map.rightIntake.set(ControlMode.PercentOutput, .39);
        }

        // If intaking (present tense), and the rezero button is pressed, then rezero
        else if (reZeroIntake) {
            Map.intakeExtend.set(ControlMode.PercentOutput, -.3);
            // Limit switch at zero position -> HOME the values
            if (Map.intakeStop.get()) {
                Map.intakeExtend.set(ControlMode.PercentOutput, 0);
                Map.intakeExtend.setSelectedSensorPosition(0);
            }
        }

        // Move to general shooting position and enable the ability to shoot
        else if (trippleToggle == positionShoot) {
            Map.intakeExtend.set(ControlMode.Position, 30000);

            // If the elevator is UP, we are scoring in the AMP, so we OUTTAKE the
            //   gamepiece rather than feed it into the shooter mechanism
            if (Elevator.toggleAmpScore && Elevator.toggleOuttake) {
                Map.leftIntake.set(ControlMode.PercentOutput, -1);
                Map.rightIntake.set(ControlMode.PercentOutput, -1);
            }
            // If the shooter wheels are up to speed, then automatically insert
            //  the note into the shooter if the `launchNote` button is pressed
            //  NOTE: At present, the `launchNote` button is the SAME as the Launcher.launch button
            else if (launchNote && Map.leftLauncher.getSelectedSensorVelocity() > 16300) {
                Map.leftIntake.set(ControlMode.PercentOutput, 1);
                Map.rightIntake.set(ControlMode.PercentOutput, 1);
            }
            // If the shooter wheels are NOT up to speed, manually control the intake/outtake
            else if (Map.leftLauncher.getSelectedSensorVelocity() < 15400) {
                Map.leftIntake.set(ControlMode.PercentOutput, (intakeAxis - outtakeAxis));
                Map.rightIntake.set(ControlMode.PercentOutput, -(outtakeAxis - intakeAxis) * 1.4);
            }
            // Other cases, we just sit on our hands
            else {
                
            }
        } 
        
        // Move the intake to the zero position
        else if (trippleToggle == positionZero) {

            // Move the intake to zero position
            Map.intakeExtend.set(ControlMode.Position, 0);

            // Re-zero intake if limit switch is pressed
            if (Map.intakeStop.get()) {
                Map.intakeExtend.setSelectedSensorPosition(0);
            }

            // If the elevator is UP, we are scoring in the AMP, so we OUTTAKE the
            //   gamepiece rather than feed it into the shooter mechanism
            if (Elevator.toggleAmpScore && Elevator.toggleOuttake) {
                Map.leftIntake.set(ControlMode.PercentOutput, -1);
                Map.rightIntake.set(ControlMode.PercentOutput, -1);
            }
            // If the shooter wheels are up to speed, then automatically insert
            //  the note into the shooter if the `launchNote` button is pressed
            //  NOTE: At present, the `launchNote` button is the SAME as the Launcher.launch button
            else if (launchNote && Map.leftLauncher.getSelectedSensorVelocity() > 16000) {
                Map.leftIntake.set(ControlMode.PercentOutput, 1);
                Map.rightIntake.set(ControlMode.PercentOutput, 1);
            }
            // If the shooter wheels are NOT up to speed, manually control the intake/outtake
            else if (Map.leftLauncher.getSelectedSensorVelocity() < 15400) {
                Map.leftIntake.set(ControlMode.PercentOutput, (intakeAxis - outtakeAxis));
                Map.rightIntake.set(ControlMode.PercentOutput, -(outtakeAxis - intakeAxis) * 1.4);
            }
            // Other cases, we just sit on our hands
            else {}
        }
    }

    
}
