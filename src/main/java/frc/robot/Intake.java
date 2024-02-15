package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class represents an intake mechanism for a robot.
 */
public class Intake {
    public static PIDController intakePID = new PIDController(.00003, 0.00, 0.000001);
    private static boolean toggleOut = false;
    private static boolean toggleScore = false;
    private static int trippleToggle = 1;

    /**
     * Initializes the intake mechanism.
     */
    public static void init() {
        toggleOut = false;
        toggleScore = false;
        Map.movementIntake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.intakeRight.setInverted(true);
        Map.movementIntake.setSelectedSensorPosition(0);
        Map.movementIntake.setNeutralMode(NeutralMode.Brake);
        trippleToggle = 1;
    }

    public static void disable(boolean button) {
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

    public static void run(boolean button, boolean scoreButton) {
        intakExtension(button);
        intakeSpin();
        score(scoreButton);
    }

    public static boolean intakExtension(boolean button) {
        boolean extend = false;
        if (Map.intakeStop.get()) {
            Map.movementIntake.setSelectedSensorPosition(0);

        }
        // chang this.
        // only extends at 45 so it can't go past frame perimiter.
        // change this
        if (Map.launcherPivot.getSelectedSensorPosition() == -21000) {
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
                Map.movementIntake.set(ControlMode.PercentOutput,
                        intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 99600));
            }
        } else if (extend == false) {
            if (Map.movementIntake.getSupplyCurrent() > 5) {
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
            } // change this to out position
            else {
                // 45 change this
                if (Map.launcherPivot.getSelectedSensorPosition() < -21000) {
                    Map.movementIntake.set(ControlMode.PercentOutput,
                            intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 36400));
                } else if (Map.launcherPivot.getSelectedSensorPosition() > -21000) {
                    Map.movementIntake.set(ControlMode.PercentOutput,
                            intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 36400));

                }
            }
        }
        return extend;
    }

    /**
     * Spins the intake wheels based on an axis input and the extension state of the
     * intake mechanism.
     * 
     * @param axis     The axis input to control the speed of the intake wheels.
     * @param extended The state of the intake mechanism (extended or retracted).
     */

    public static void intakeSpin() {

        if (Map.movementIntake.getSelectedSensorPosition() > 80000) {
            Map.intakeRight.set(ControlMode.PercentOutput, .4);
            Map.intakeLeft.set(ControlMode.PercentOutput, .4);
            if (Map.lightStop.get()) {
                Map.intakeRight.set(ControlMode.PercentOutput, 0);
                Map.intakeLeft.set(ControlMode.PercentOutput, 0);
            }
        } else {
            Map.intakeRight.set(ControlMode.PercentOutput, .0);
            Map.intakeLeft.set(ControlMode.PercentOutput, .0);
        }
    }

    public static void test(boolean toggle1, boolean toggle2) {
    SmartDashboard.putNumber("intakeZone", Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition()) - 21900) );
        if (toggle2) {
            toggleScore = !toggleScore;
        }
        if (toggle1) {
            toggleOut = !toggleOut;
            if (Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition()) - 21900) > 1000) {
                toggleOut = false;
            }
        }

        if (toggle1 == true && Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition()) - 21900) < 1000 && Map.lightStop.get()==false) {
            trippleToggle = 2;

        } else if (toggle1 == false || Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition()) - 21900) > 1000 || Map.lightStop.get()==true) {
            if (toggleScore == true) {
                trippleToggle = 3;

            } else if (toggleScore == false) {
                trippleToggle = 1;
            }
            Map.movementIntake.set(ControlMode.PercentOutput, 0);
        }
        if (trippleToggle == 2) {
            Map.movementIntake.set(ControlMode.PercentOutput,
                    intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 80000));
        }

       else if (trippleToggle == 3) {
            Map.movementIntake.set(ControlMode.PercentOutput,
                    intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 37000));
        } else if (trippleToggle == 1) {
            Map.movementIntake.set(ControlMode.PercentOutput,
                    intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 0));

            if (Map.intakeStop.get()) {
                Map.movementIntake.setSelectedSensorPosition(0);
            }
        }

    }

    /**
     * Scores the intake mechanism based on a button input.
     * 
     * @param button The button input to control the scoring of the intake
     *               mechanism.
     */
    public static void score(boolean button) {
        // change this
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
                Map.intakeRight.set(ControlMode.PercentOutput, 1);
                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
            } else {
                Map.intakeRight.set(ControlMode.PercentOutput, 0);
                Map.intakeLeft.set(ControlMode.PercentOutput, 0);
            }
        }
    }
}
