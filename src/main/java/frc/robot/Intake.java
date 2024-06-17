package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    public static PositionDutyCycle intakeExtend = new PositionDutyCycle(48.8);
    public static boolean toggleScore = false;
    public static int trippleToggle = 1;
    public static int a = 0;
    public static double robotToTagZ;
    public static int positionIntake = 2;
    public static int positionFix = 3;
    public static int positionZero = 1;
    public static Slot0Configs intakePID = new Slot0Configs();

    public static void init() {
        // toggleOut = false;
        a = 0;
        toggleScore = false;

        Map.intakeTop.setInverted(false);

        Map.intakeExtend.setPosition(0);
        Map.intakeExtend.getConfigurator().apply(Map.brake);
        trippleToggle = 1;
        intakePID.kP = 5.101;
        intakePID.kI = .0;
        intakePID.kD = .001;
        Map.intakeExtend.getConfigurator().apply(intakePID);
      
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
            Map.intakeExtend.getConfigurator().apply(Map.coast);
        } else {
            Map.intakeExtend.getConfigurator().apply(Map.brake);
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
    public static void run(boolean intakePositionButton, boolean launchNote, boolean reZeroIntake,
            double intakeAxis, double outtakeAxis, boolean autoZero, boolean red, boolean fix) {
        // put number to smart dashboard
        SmartDashboard.putNumber("intakeZone", Launcher.distanceFrom45());

        // If "make it go out" button is pressed, and the pivot (Launcher) is in
        // the correct position, and the beam-break is NOT triggered, set the
        // tripple toggle to positionIntake
        if (intakePositionButton == true && Map.lightStop.get() == false) {
            trippleToggle = positionIntake;
        } 
        // If AUTO hold-zero OR the pivot is steeper than 50º
        else if (autoZero /*
                           * || * Launcher.pivotEncoder.getAbsolutePosition() > 50
                           */) {
            trippleToggle = positionZero;
        } else if (fix){
            trippleToggle = positionFix;
        }
        // If we WANT to be in scoring position and the intake is NOT too far in (ticks
        // are backwards), set triple toggle to positionShoot

        // Default fail-safe is zero position
        else {
            trippleToggle = positionZero;
        }

        // This is where we actually do stuff!!!å

        // Move to intake position & turn on the intake rollers (leftIntake &
        // rightIntake)
        if (trippleToggle == positionIntake) {
            // needs to change for v6
            Map.intakeExtend.setControl(intakeExtend);

            if (Map.lightStop.get()) {
                Map.intakeBottom.set(ControlMode.PercentOutput, 0);
                Map.intakeTop.set(ControlMode.PercentOutput, 0);

            } else {
                Map.intakeBottom.set(ControlMode.PercentOutput, .35);
                Map.intakeTop.set(ControlMode.PercentOutput, .3);
            }
        }

        // If intaking (present tense), and the rezero button is pressed, then rezero

       else if (trippleToggle==positionFix){
        intakeExtend.Slot = 0;
         Map.intakeExtend.setControl(intakeExtend);
       } // Move the intake to the zero position
        else if (trippleToggle == positionZero) {
            if (Map.intakeStop.get()) {
                Map.intakeExtend.setControl(new DutyCycleOut( 0));
                Map.intakeExtend.setPosition(0);
            } else {
                if (Map.intakeExtend.getPosition().getValueAsDouble() > 4.8) {
                    Map.intakeExtend.setControl(new DutyCycleOut( -.8));
                } else {
                    Map.intakeExtend.setControl(new DutyCycleOut( -.3));
                }
            }

            // Move the intake to zero position
            // Map.intakeExtend.setControl(new DutyCycleOut( -.6);
            // Map.intakeExtend.set(ControlMode.Position, 0);

            // Re-zero intake if limit switch is pressed
            // if (Map.intakeStop.get()) {
            // Map.intakeExtend.setControl(new DutyCycleOut( 0);
            // Map.intakeExtend.setPosition(0)
            // }

            // If the elevator is UP, we are scoring in the AMP, so we OUTTAKE the
            // gamepiece rather than feed it into the shooter mechanism
            if (Elevator.toggleAmpScore) {
                if (a == 0) {
                    Map.intakeBottom.set(ControlMode.PercentOutput, .25);
                    Map.intakeTop.set(ControlMode.PercentOutput, .25);
                    if (Map.lightStop.get() == false) {
                        a = 1;
                    }
                } else if (a == 1) {
                    Map.intakeBottom.set(ControlMode.PercentOutput, .25);
                    Map.intakeTop.set(ControlMode.PercentOutput, .25);
                    if (Map.lightStop.get()) {
                        Map.intakeBottom.set(ControlMode.PercentOutput, .0);
                        Map.intakeTop.set(ControlMode.PercentOutput, .0);
                        a = 2;
                    }
                } else if (a == 2) {
                    if (Map.elevatorTop.get()) {
                        Map.intakeBottom.set(ControlMode.PercentOutput, -1);
                        Map.intakeTop.set(ControlMode.PercentOutput, -1);
                    } else {
                        Map.intakeBottom.set(ControlMode.PercentOutput, -0);
                        Map.intakeTop.set(ControlMode.PercentOutput, -0);

                    }

                }

            }
            // If the shooter wheels are up to speed, then automatically insert
            // the note into the shooter if the `launchNote` button is pressed
            // NOTE: At present, the `launchNote` button is the SAME as the Launcher.launch
            // button
            else if (launchNote && Map.topLauncher.getVelocity().getValueAsDouble() > 68.36) {
              a = 0;
                Map.intakeBottom.set(ControlMode.PercentOutput, 1 + (intakeAxis - outtakeAxis));
                Map.intakeTop.set(ControlMode.PercentOutput, 1 - (outtakeAxis - intakeAxis) * 1.4);
            }
            // If the shooter wheels are NOT up to speed, manually control the
            // intake/outtake
            else if (Map.bottomLauncher.getVelocity().getValueAsDouble() < 14.65) {
              a = 0;
                Map.intakeBottom.set(ControlMode.PercentOutput, (intakeAxis - outtakeAxis));
                Map.intakeTop.set(ControlMode.PercentOutput, -(outtakeAxis - intakeAxis) * 1.4);
            }
            // Other cases, we just sit on our hands
            else {
                a = 0;
            }
        }
    }

}
