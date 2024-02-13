package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * This class represents an intake mechanism for a robot.
 */
public class Intake {

    /**
     * Initializes the intake mechanism.
     */
    public static void init() {
        Map.intakeRight.setInverted(true);
        Map.movementIntake.setSelectedSensorPosition(0);
    }

    /**
     * Runs the intake mechanism based on a button input.
     * 
     * @param button The button input to control the intake mechanism.
     */
    public static void run(boolean button) {}

    /**
     * Extends or retracts the intake mechanism based on a button input.
     * 
     * @param button The button input to control the extension/retraction of the intake mechanism.
     * @return true if the intake mechanism is extended, false otherwise.
     */
    public static boolean intakExtension(boolean button) {
        boolean extend = false;
        // chang this.
        // only extends at 45 so it can't go past frame perimiter.
        if (Map.launcherPivot.getSelectedSensorPosition() == 45) {
            // toggle switch for extension
            if (button) {
                extend = !extend;
            }
        } else {
            extend = false;
        }

        if (extend == true) {
            // extension
            // stops if current is to high
            // change supply current
            if (Map.movementIntake.getSupplyCurrent() > 5) {
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
            } // change this to out position
            else {
                Map.movementIntake.set(ControlMode.Position, 30);
            }
        } else if (extend == false) {
            if (Map.movementIntake.getSupplyCurrent() > 5) {
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
            } // change this to out position
            else {
                Map.movementIntake.set(ControlMode.Position, 0);
            }
        }
        return extend;
    }

    /**
     * Spins the intake wheels based on an axis input and the extension state of the intake mechanism.
     * 
     * @param axis     The axis input to control the speed of the intake wheels.
     * @param extended The state of the intake mechanism (extended or retracted).
     */
    public static void intakeSpin(double axis, boolean extended) {
        if (axis > .13) {
            Map.intakeRight.set(ControlMode.PercentOutput, axis);
            Map.intakeLeft.set(ControlMode.PercentOutput, axis);
        } else if (extended) {
            Map.intakeRight.set(ControlMode.PercentOutput, .5);
            Map.intakeLeft.set(ControlMode.PercentOutput, .5);
        } else {
            Map.intakeRight.set(ControlMode.PercentOutput, .0);
            Map.intakeLeft.set(ControlMode.PercentOutput, .0);
        }
    }

    /**
     * Scores the intake mechanism based on a button input.
     * 
     * @param button The button input to control the scoring of the intake mechanism.
     */
    public static void score(boolean button) {
        // change this
        if (Map.rightElevator.getSelectedSensorPosition() > 1000) {
            if (button) {
                Map.intakeRight.set(ControlMode.PercentOutput, -.5);
                Map.intakeLeft.set(ControlMode.PercentOutput, -.5);
            } else {
                Map.intakeRight.set(ControlMode.PercentOutput, 0);
                Map.intakeLeft.set(ControlMode.PercentOutput, 0);
            }
        } else {
            if (button) {
                Map.intakeRight.set(ControlMode.PercentOutput, 1);
                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
            } else {
                Map.intakeRight.set(ControlMode.PercentOutput, 0);
                Map.intakeLeft.set(ControlMode.PercentOutput, 0);
            }
        }
    }
}
