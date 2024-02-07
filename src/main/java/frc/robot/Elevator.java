package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Vision.RaspberryPi;

public class Elevator {
    public static void elevatorInit() {

        Map.rightElevator.config_kF(0, 0.048);
        Map.leftElevator.config_kF(0, 0.048);
        Map.rightElevator.config_kP(0, 0.4);
        Map.leftElevator.config_kP(0, 0.4);
        Map.rightElevator.config_kI(0, 0.001);
        Map.leftElevator.config_kI(0, 0.001);
        Map.rightElevator.config_IntegralZone(0, 300);
        Map.leftElevator.config_IntegralZone(0, 300);

        Map.rightElevator.setNeutralMode(NeutralMode.Brake);
        Map.leftElevator.setNeutralMode(NeutralMode.Brake);

        Map.rightElevator.setInverted(true);
    }

    public static void goToPos(boolean button1, boolean button2, boolean button3, boolean button4, double axis) {
        // change these
        boolean toggle1 = true;
        boolean toggle2 = false;
        boolean toggle3 = false;
        boolean toggle4 = false;
        double pos1 = 0;
        double pos2 = 27;
        double pos3 = 42;
        // change this
        int upperLimit = 1;
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
                        //change number for supply current
                        if (Map.rightElevator.getSupplyCurrent() > 10 || Map.leftElevator.getSupplyCurrent() > 10) {
                            Map.rightElevator.set(ControlMode.PercentOutput, 0);
                            Map.leftElevator.set(ControlMode.PercentOutput, 0);

                        } else {
                            Map.rightElevator.set(ControlMode.PercentOutput, axis/2);
                            Map.leftElevator.set(ControlMode.PercentOutput, axis/2);

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
