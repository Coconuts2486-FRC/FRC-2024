package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

//*This is an intake
public class Intake {
    public static void init() {
        Map.intakeRight.setInverted(true);
        Map.movementIntake.setSelectedSensorPosition(0);

    }
    public static void run(boolean button){
        

    }

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

    // bottom function makes it so when a button is pressed it activates using the
    // top function

    // bottom function is what is used to spin up the wheels for intaking in notes
    public static void intakeSpin(double axis, boolean extended) {

        if (axis>.13) {
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

    // bottom function is what is used to score on the amp
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
