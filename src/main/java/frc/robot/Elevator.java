/**
 * The Elevator class controls the elevator mechanism of the robot.
 * It provides methods to initialize and run the elevator.
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Vision.RaspberryPi;

public class Elevator {

    /**
     * The PIDController used for elevator position control.
     */
    public static PIDController elevatorPID = new PIDController(
        .00008,
        0.00,
        0.000001
    );

    /**
     * Initializes the elevator by setting the neutral mode, sensor positions, and inversion.
     */
    public static void init() {
        Map.rightElevator.setNeutralMode(NeutralMode.Brake);
        Map.leftElevator.setNeutralMode(NeutralMode.Brake);
        Map.rightElevator.setSelectedSensorPosition(0, 0, 0);
        Map.leftElevator.setSelectedSensorPosition(0, 0, 0);

        Map.rightElevator.setInverted(true);
    }

    /**
     * Runs the elevator based on the button inputs and axis value.
     * 
     * @param button1 The state of button 1.
     * @param button2 The state of button 2.
     * @param button3 The state of button 3.
     * @param button4 The state of button 4.
     * @param axis The value of the elevator axis.
     */
    public static void run(
        boolean button1,
        boolean button2,
        boolean button3,
        boolean button4,
        double axis
    ) {
        // change these
        boolean toggle1 = true;
        boolean toggle2 = false;
        boolean toggle3 = false;
        boolean toggle4 = false;
        double pos1 = 0;
        double pos2 = -60000;
        double pos3 = -84000;
        // change this
        Map.rightElevator.follow(Map.leftElevator);
        int upperLimit = -84000;
        if (button1) {
            toggle1 = true;
            toggle2 = false;
            toggle3 = false;
            toggle4 = false;

            if (toggle1) {
                Map.leftElevator.set(
                    ControlMode.PercentOutput,
                    elevatorPID.calculate(
                        Map.leftElevator.getSelectedSensorPosition(),
                        pos1
                    )
                );
            }
        }
        if (button2) {
            toggle1 = false;
            toggle2 = true;
            toggle3 = false;
            toggle4 = false;
            if (toggle2) {
                Map.leftElevator.set(
                    ControlMode.PercentOutput,
                    elevatorPID.calculate(
                        Map.leftElevator.getSelectedSensorPosition(),
                        pos2
                    )
                );
            }
        }
        if (button3) {
            toggle1 = false;
            toggle2 = false;
            toggle3 = true;
            toggle4 = false;
            if (toggle3) {
                Map.leftElevator.set(
                    ControlMode.Position,
                    elevatorPID.calculate(
                        Map.leftElevator.getSelectedSensorPosition(),
                        pos3
                    )
                );
            }
        }
        if (button4) {
            toggle1 = false;
            toggle2 = false;
            toggle3 = true;
            toggle4 = false;
            if (toggle4) {
                if (
                    Map.elevatorBottom.DIO() &&
                    Map.leftElevator.getSelectedSensorVelocity() < 0
                ) {
                    Map.leftElevator.set(ControlMode.PercentOutput, 0);
                } else {
                    Map.leftElevator.set(ControlMode.PercentOutput, axis);
                }
            }
        }
        if (Map.elevatorTop.DIO()) {
            Map.launcherPivot.setSelectedSensorPosition(upperLimit);
        }
        if (Map.elevatorBottom.DIO()) {
            Map.launcherPivot.setSelectedSensorPosition(0);
        }
    }
}
