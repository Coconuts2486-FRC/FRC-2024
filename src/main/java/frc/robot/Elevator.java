/**
 * The Elevator class controls the elevator mechanism of the robot.
 * It provides methods to initialize and run the elevator.
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("removal")
public class Elevator {
    public static boolean toggleClimbUp = false;
    public static boolean toggleAmpScore = false;
    public static boolean readyToScore = false;
    public static boolean toggleDown = false;
    public static boolean toggleOuttake = false;
    public static boolean toggleUp = false;
    public static int a = 0;

    // The PIDController used for elevator position control.
    public static PIDController elevatorPID = new PIDController(
            .0001,
            0.000001,
            0.00000);

    /**
     * Initializes the elevator by setting the neutral mode, sensor positions, and
     * inversion.
     */
    public static void init() {
       //// Map.leftElevator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.rightElevator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        Map.rightElevator.setNeutralMode(NeutralMode.Brake);
       //// Map.leftElevator.setNeutralMode(NeutralMode.Brake);
        Map.rightElevator.setSelectedSensorPosition(0);
       //// Map.leftElevator.setSelectedSensorPosition(0);

        Map.rightElevator.setInverted(false);
        toggleClimbUp = false;
        toggleAmpScore = false;
        toggleDown = false;
        toggleOuttake = false;
        toggleUp = false;
        readyToScore = false;

    }

    /**
     * Disable the Elevator with a button press
     * 
     * @param button The disable button
     */
    public static void disable(boolean button) {

        toggleClimbUp = false;
        if (button) {
           //// Map.leftElevator.setNeutralMode(NeutralMode.Coast);
            Map.rightElevator.setNeutralMode(NeutralMode.Coast);
        }

        else {
           //// Map.leftElevator.setNeutralMode(NeutralMode.Brake);
            Map.rightElevator.setNeutralMode(NeutralMode.Brake);
        }
    }

    /**
     * @param toggleUpForClimb
     * 
     * @param toggleUpForAmp
     * @param manualUpAxis
     * @param manualDownAxis
     */
    public static void run(boolean toggleUpForClimb, boolean toggleUpForAmp, double manualUpAxis,
            double manualDownAxis, boolean kill) {
        
        if (toggleUpForClimb) {
            toggleUp = !toggleUp;
            toggleDown = true;
          //  toggleAmpScore = false;
        }
        if (toggleUpForAmp) {
            toggleUp = !toggleUp;
            toggleDown = true;
            toggleAmpScore = true;
        }

        if (kill){
                       Map.rightElevator.set(ControlMode.PercentOutput, 0);
               //// Map.leftElevator.set(ControlMode.PercentOutput, 0);
        }
        if (toggleUp) {

            toggleDown = true;

              Map.rightElevator.set(ControlMode.PercentOutput, -.90);
           //// Map.leftElevator.set(ControlMode.PercentOutput, -.90);

            if (Map.elevatorTop.get()) {
                           Map.rightElevator.set(ControlMode.PercentOutput, 0);
               //// Map.leftElevator.set(ControlMode.PercentOutput, 0);
                if (toggleAmpScore){
                    
                    readyToScore = true;
                    
                }

                ////// Map.leftElevator.setSelectedSensorPosition(-85000);
            }

        } else if (toggleDown == true) {
            a = 0;
            toggleAmpScore = false;
            Map.rightElevator.set(ControlMode.PercentOutput, .65);

           //// Map.leftElevator.set(ControlMode.PercentOutput, .65);
            if (Map.elevatorBottom.get()) {
     
                 Map.rightElevator.set(ControlMode.PercentOutput, 0);
               //// Map.leftElevator.set(ControlMode.PercentOutput, 0);
                ////// Map.leftElevator.setSelectedSensorPosition(-85000);
                toggleDown = false;
                readyToScore = false;
            }
           

        } else {
            if (manualUpAxis > .1) {
                Map.rightElevator.set(ControlMode.PercentOutput, -manualUpAxis);
               //// Map.leftElevator.set(ControlMode.PercentOutput, -manualUpAxis);
                if (Map.elevatorTop.get()) {
                    Map.rightElevator.set(ControlMode.PercentOutput, 0);
                   //// Map.leftElevator.set(ControlMode.PercentOutput, 0);
                }
            } else if (manualDownAxis > .1) {
                Map.rightElevator.set(ControlMode.PercentOutput, manualDownAxis);
               //// Map.leftElevator.set(ControlMode.PercentOutput, manualDownAxis);
                if (Map.elevatorBottom.get()) {
                    Map.rightElevator.set(ControlMode.PercentOutput, 0);
                   //// Map.leftElevator.set(ControlMode.PercentOutput, 0);
                }
            } else {
                Map.rightElevator.set(ControlMode.PercentOutput, 0);
               //// Map.leftElevator.set(ControlMode.PercentOutput, 0);
            }
        }
    }

public static void manualRun(boolean toggleUpForClimb, boolean toggleUpForAmp, double manualUpAxis,
            double manualDownAxis, boolean kill) {


           if (toggleUpForAmp) {
            toggleAmpScore = true;
        } else {
            toggleAmpScore = false;
        }

        if (kill){
                       Map.rightElevator.set(ControlMode.PercentOutput, 0);
               //// Map.leftElevator.set(ControlMode.PercentOutput, 0);
        }
        if (toggleUpForAmp) {

           

              Map.rightElevator.set(ControlMode.PercentOutput, -.90);
           //// Map.leftElevator.set(ControlMode.PercentOutput, -.90);

            if (Map.elevatorTop.get()) {
                           Map.rightElevator.set(ControlMode.PercentOutput, 0);
               //// Map.leftElevator.set(ControlMode.PercentOutput, 0);
                if (toggleAmpScore){
                    readyToScore = true;
                }

                ////// Map.leftElevator.setSelectedSensorPosition(-85000);
            }

        } else if (toggleUpForClimb) {
              

              Map.rightElevator.set(ControlMode.PercentOutput, -.90);
           //// Map.leftElevator.set(ControlMode.PercentOutput, -.90);

            if (Map.elevatorTop.get()) {
                           Map.rightElevator.set(ControlMode.PercentOutput, 0);
               //// Map.leftElevator.set(ControlMode.PercentOutput, 0);
            
                //// Map.leftElevator.setSelectedSensorPosition(-85000);
            }


        } else if (Map.elevatorBottom.get()){
            init();

            if (manualUpAxis > .1) {
                Map.rightElevator.set(ControlMode.PercentOutput, -manualUpAxis);
               // Map.leftElevator.set(ControlMode.PercentOutput, -manualUpAxis);
                if (Map.elevatorTop.get()) {
                    Map.rightElevator.set(ControlMode.PercentOutput, 0);
                   // Map.leftElevator.set(ControlMode.PercentOutput, 0);
                }
            } else if (manualDownAxis > .1) {
                Map.rightElevator.set(ControlMode.PercentOutput, manualDownAxis);
               // Map.leftElevator.set(ControlMode.PercentOutput, manualDownAxis);
                if (Map.elevatorBottom.get()) {
                    Map.rightElevator.set(ControlMode.PercentOutput, 0);
                   // Map.leftElevator.set(ControlMode.PercentOutput, 0);
                }
            } else {
                Map.rightElevator.set(ControlMode.PercentOutput, 0);
               // Map.leftElevator.set(ControlMode.PercentOutput, 0);
            }

        }else{
            
             if (manualUpAxis > .1) {
                Map.rightElevator.set(ControlMode.PercentOutput, -manualUpAxis);
               // Map.leftElevator.set(ControlMode.PercentOutput, -manualUpAxis);
                if (Map.elevatorTop.get()) {
                    Map.rightElevator.set(ControlMode.PercentOutput, 0);
                   // Map.leftElevator.set(ControlMode.PercentOutput, 0);
                }
            } else if (manualDownAxis > .1) {
                Map.rightElevator.set(ControlMode.PercentOutput, manualDownAxis);
               // Map.leftElevator.set(ControlMode.PercentOutput, manualDownAxis);
                if (Map.elevatorBottom.get()) {
                    Map.rightElevator.set(ControlMode.PercentOutput, 0);
                   // Map.leftElevator.set(ControlMode.PercentOutput, 0);
                }
            } else {
                Map.rightElevator.set(ControlMode.PercentOutput, .80);
               // Map.leftElevator.set(ControlMode.PercentOutput, .80);
            }
        
        } 
    }
}


