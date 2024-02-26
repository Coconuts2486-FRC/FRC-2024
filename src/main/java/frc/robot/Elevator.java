/**
 * The Elevator class controls the elevator mechanism of the robot.
 * It provides methods to initialize and run the elevator.
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.RaspberryPi;

public class Elevator {
    public static boolean toggleUp = false;
    public static boolean toggleScore = false;
    /**
     * The PIDController used for elevator position control.
     */
    public static PIDController elevatorPID = new PIDController(
            .0001,
            0.000001,
            0.00000);

    /**
     * Initializes the elevator by setting the neutral mode, sensor positions, and
     * inversion.
     */
    public static void init() {
        Map.leftElevator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.rightElevator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.rightElevator.setNeutralMode(NeutralMode.Brake);
        Map.leftElevator.setNeutralMode(NeutralMode.Brake);
        Map.rightElevator.setSelectedSensorPosition(0, 0, 0);
        Map.leftElevator.setSelectedSensorPosition(0, 0, 0);

        Map.rightElevator.setInverted(true);
        toggleUp = false;
        toggleScore = false;

    }
    public static void disable(boolean button){
       
       toggleUp = false;
        if (button){      Map.leftElevator.setNeutralMode(NeutralMode.Coast);
             Map.rightElevator.setNeutralMode(NeutralMode.Coast);
        }
        
        else{
                Map.leftElevator.setNeutralMode(NeutralMode.Brake);
                Map.rightElevator.setNeutralMode(NeutralMode.Brake);
        }
    }
    public static void test(boolean button1, double axis, boolean button2, boolean button3) {
        SmartDashboard.putNumber("EVelocity", Map.leftElevator.getSelectedSensorVelocity());
       
        if (button1) {
            toggleUp = !toggleUp;
        }
          if (button2) {
            toggleScore = !toggleScore;
        }


        if (toggleUp == true) {

           

            Map.rightElevator.set(ControlMode.PercentOutput,-.45);
                   // elevatorPID.calculate(Map.leftElevator.getSelectedSensorPosition(), -85000));
                       

            Map.leftElevator.follow(Map.rightElevator);

            if (Map.elevatorTop.get()) {
                Map.rightElevator.set(ControlMode.PercentOutput,0);
              //  Map.leftElevator.setSelectedSensorPosition(-85000);
            }

            
        } else if (toggleScore == true){
             Map.rightElevator.set(ControlMode.PercentOutput,
                    elevatorPID.calculate(Map.leftElevator.getSelectedSensorPosition(), -85000));
                    
            Map.leftElevator.follow(Map.rightElevator);
            if(Map.leftElevator.getSelectedSensorPosition()<-80000 ){
                Map.intakeLeft.set(ControlMode.PercentOutput,-1);
                    Map.intakeRight.set(ControlMode.PercentOutput,1);
                    Timer.delay(.28);
                    toggleScore = false;
            }
            
                    
                       if (Map.elevatorTop.get()) {
                Map.rightElevator.set(ControlMode.PercentOutput,0);
                Map.leftElevator.follow(Map.rightElevator);
              //  Map.leftElevator.setSelectedSensorPosition(-85000);
            }
        } else if (button3){
             Map.rightElevator.set(ControlMode.PercentOutput,
                    elevatorPID.calculate(Map.leftElevator.getSelectedSensorPosition(), -39000));
                    
            Map.leftElevator.follow(Map.rightElevator);
        }

       else if (toggleUp == false&&toggleScore== false && button3 == false) {

            Map.rightElevator.set(ControlMode.PercentOutput,
                    elevatorPID.calculate(Map.rightElevator.getSelectedSensorPosition(), 0));
            Map.leftElevator.follow(Map.rightElevator);

            Map.launcherPivot.set(ControlMode.PercentOutput, 0);
            if (Map.elevatorBottom.get()) {
                Map.leftElevator.setSelectedSensorPosition(0);
                 Map.rightElevator.setSelectedSensorPosition(0);
                       Map.rightElevator.set(ControlMode.PercentOutput,0);
            }
        }
    }

    /**
     * Runs the elevator based on the button inputs and axis value.
     * 
     * @param button1 The state of button 1.
     * @param button2 The state of button 2.
     * @param button3 The state of button 3.
     * @param button4 The state of button 4.
     * @param axis    The value of the elevator axis.
     */
    public static void run(boolean button1, boolean button2, boolean button3, boolean button4, double axis) {
        // change these

        boolean toggle1 = true;
        boolean toggle2 = false;
        boolean toggle3 = false;
        boolean toggle4 = false;
        double pos1 = 0;
        double pos2 = -60000;
        double pos3 = -30000;

        if (Map.elevatorTop.get()) {
            Map.leftElevator.set(ControlMode.PercentOutput, 0);
        }

        // change this
        Map.rightElevator.follow(Map.leftElevator);
        int upperLimit = -84000;
        if (button1) {
            toggle1 = true;
            toggle2 = false;
            toggle3 = false;
            toggle4 = false;
        }

        if (button2) {
            toggle1 = false;
            toggle2 = true;
            toggle3 = false;
            toggle4 = false;
        }

        if (button3) {
            toggle1 = false;
            toggle2 = false;
            toggle3 = true;
            toggle4 = false;
        }
        if (button4) {
            toggle1 = false;
            toggle2 = false;
            toggle3 = false;
            toggle4 = true;
        }

        if (toggle1) {
            double output = elevatorPID.calculate(Map.leftElevator.getSelectedSensorPosition(), pos1);
            SmartDashboard.putNumber("output", output);
            Map.leftElevator.set(ControlMode.PercentOutput,
                    elevatorPID.calculate(Map.leftElevator.getSelectedSensorPosition(), pos1));
            if (Map.elevatorBottom.get()) {
                Map.leftElevator.setSelectedSensorPosition(pos1);
            }
        }

        if (toggle2) {
            double output = elevatorPID.calculate(Map.leftElevator.getSelectedSensorPosition(), pos2);
            SmartDashboard.putNumber("output", output);
            Map.leftElevator.set(ControlMode.PercentOutput,
                    elevatorPID.calculate(Map.leftElevator.getSelectedSensorPosition(), pos2));
        }

        if (toggle3) {
            double output = elevatorPID.calculate(Map.leftElevator.getSelectedSensorPosition(), pos3);
            SmartDashboard.putNumber("output", output);
            Map.leftElevator.set(ControlMode.PercentOutput,
                    elevatorPID.calculate(Map.leftElevator.getSelectedSensorPosition(), pos3));
        }

        if (toggle4) {

            if (Map.elevatorBottom.get()) {
                Map.leftElevator.set(ControlMode.PercentOutput, 0);

            } else {
                Map.leftElevator.set(ControlMode.PercentOutput, axis);

            }
        }

        if (Map.elevatorTop.get()) {
            Map.leftElevator.setSelectedSensorPosition(upperLimit);
        }
        if (Map.elevatorBottom.get()) {
            Map.leftElevator.setSelectedSensorPosition(0);

        }

    }
}