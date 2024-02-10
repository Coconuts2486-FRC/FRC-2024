package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Vision.RaspberryPi;

public class Elevator {
    public static void elevatorInit() {

     
        Map.rightElevator.setNeutralMode(NeutralMode.Brake);
        Map.leftElevator.setNeutralMode(NeutralMode.Brake);
         Map.rightElevator.setSelectedSensorPosition(0);
        Map.leftElevator.setSelectedSensorPosition(0);

        Map.rightElevator.setInverted(true);
    }

    public static void goToPos(boolean button1, boolean button2, boolean button3, boolean button4, double axis) {
        // change these
        boolean toggle1 = true;
        boolean toggle2 = false;
        boolean toggle3 = false;
        boolean toggle4 = false;
        double pos1 = 0;
        double pos2 = 500;
        double pos3 = 1000;
        // change this
        int upperLimit = 1000;
        if (button1) {
            toggle1 = true;
            toggle2 = false;
            toggle3 = false;
            toggle4 = false;
            if (toggle1) {
                Map.rightElevator.set(ControlMode.Position, pos1);
                Map.leftElevator.set(ControlMode.Position, pos1);
            }
        }
        if (button2) {
            toggle1 = false;
            toggle2 = true;
            toggle3 = false;
            toggle4 = false;
            if (toggle2) {
                Map.rightElevator.set(ControlMode.Position, pos2);
                Map.leftElevator.set(ControlMode.Position, pos2);
            }
        }
        if (button3) {
            toggle1 = false;
            toggle2 = false;
            toggle3 = true;
            toggle4 = false;
            if (toggle3) {
                Map.rightElevator.set(ControlMode.Position, pos3);
                Map.leftElevator.set(ControlMode.Position, pos3);
            }
        }
                if (button4) {
                    toggle1 = false;
                    toggle2 = false;
                    toggle3 = true;
                    toggle4 = false;
                    if (toggle4) {
         
                        if (Map.elevatorBottom.DIO() && Map.leftElevator.getSelectedSensorVelocity()<0) {
                            Map.rightElevator.set(ControlMode.PercentOutput, 0);
                            Map.leftElevator.set(ControlMode.PercentOutput, 0);

                        } else {
                            Map.rightElevator.set(ControlMode.PercentOutput, axis);
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
