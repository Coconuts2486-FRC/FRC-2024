package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class represents an intake mechanism for a robot.
 */
public class Intake {
    public static PIDController intakePID = new PIDController(.00007, 0.00000, 0.00000093);
    private static boolean toggleOut = false;
    private static boolean toggleScore = false;
    private static int trippleToggle = 1;
    public static double shotClock; 

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
        Map.movementIntake.config_kP(0, 0.55);
        Map.movementIntake.config_kI(0, 0.0000);
        Map.movementIntake.config_kD(0, 0.000001);
        //  Map.rightLauncher.config_kP(0,1);
        // Map.leftLauncher.config_kP(0,1);
        // Map.rightLauncher.config_kI(0,.1);
        // Map.leftLauncher.config_kI(0,.1);

        Map.leftLauncher.configOpenloopRamp(0.1);
        Map.rightLauncher.configOpenloopRamp(0.1);
        
        
        Map.rightLauncher.config_kF(0, 0.15);
        Map.leftLauncher.config_kF(0, 0.15);
        Map.rightLauncher.config_kP(0, 0.7);
        Map.leftLauncher.config_kP(0, 0.7);
        Map.rightLauncher.config_kI(0, 0.005);
        Map.leftLauncher.config_kI(0, 0.005);
        Map.rightLauncher.config_IntegralZone(0, 300);
        Map.leftLauncher.config_IntegralZone(0, 300);

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
// This is the one im using.
    public static void test(boolean toggle1, boolean toggle2,boolean button3) {
        // put number to smart dashboard
    SmartDashboard.putNumber("intakeZone", Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition()) - 23900) );
      // toggle for scoring position
        if (button3){
            shotClock = Timer.getFPGATimestamp();
        }

    if (toggle2) {
            toggleScore = !toggleScore;
        }
        if (toggle1) {
            toggleOut = !toggleOut;
            if (Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition()) - 23900) > 1000) {
                toggleOut = false;
            }
        }
            // if button, and correct position, and light sensor, set the tripple toggle to 2
        if (toggle1 == true && /*Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition()) - 23900) < 1000 && */Map.lightStop.get()==false) {
            trippleToggle = 2;
            // if any are false, set it to 3 or 1 depending on if toggle score is true or false
        } else if (toggle1 == false ||/*  Math.abs(Math.abs(Map.launcherPivot.getSelectedSensorPosition()) - 23900) > 1000 ||*/ Map.lightStop.get()==true) {
            if (toggleScore == true) {
                trippleToggle = 3;

            } else if (toggleScore == false) {
                trippleToggle = 1;
            }
            Map.movementIntake.set(ControlMode.PercentOutput, 0);
        }
        // if tripple toggle is 2, set intake to intake position and start intake
        if (trippleToggle == 2) {
            //Map.movementIntake.set(ControlMode.PercentOutput,intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(),98000));
            Map.movementIntake.set(ControlMode.Position,98000);
                    Map.intakeLeft.set(ControlMode.PercentOutput, .3);
                    Map.intakeRight.set(ControlMode.PercentOutput, -.3);
        }
        //if tripple toggle is 3, go to general shooting position and enable the ability to shoot
       else if (trippleToggle == 3) {

            //  Map.movementIntake.set(ControlMode.PercentOutput,
            //         intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 37000));
             Map.movementIntake.set(ControlMode.Position, 37000);
            
              
          //  Map.movementIntake.set(ControlMode.Position, 34500);
            // if the shoot button is pressed, check if the elevator is above 20000, and shoot out if it is, shoot if it isn't    
                       if(Robot.teleopTime - shotClock < .3){
                        
                         
                        Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                    Map.intakeRight.set(ControlMode.PercentOutput, -1);
               
                }else{
                        Map.intakeLeft.set(ControlMode.PercentOutput, 0);
                    Map.intakeRight.set(ControlMode.PercentOutput, -0);
                

                       }
                       // if toggle is 1, go to zero position
        } else if (trippleToggle == 1) {

            // Map.movementIntake.set(ControlMode.PercentOutput,
            //         intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 0));
            Map.movementIntake.set(ControlMode.Position, 0);

            //  if intake is in and elevator is up and not moving, it auto outtakes, otherwise, wait until button is pressed, and do the same as toggle 3                 
            if (Map.leftElevator.getSelectedSensorPosition()<-82700 && Map.leftElevator.getSelectedSensorVelocity()<30){
        
                Map.intakeLeft.set(ControlMode.PercentOutput, -1);
                Map.intakeRight.set(ControlMode.PercentOutput, 1);
                  
            
             } else if(button3)  {

   if(Map.leftElevator.getSelectedSensorPosition()<-20000){
                        Map.intakeLeft.set(ControlMode.PercentOutput, -1);
                    Map.intakeRight.set(ControlMode.PercentOutput, 1);
                }else if (button3) {
                        Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                    Map.intakeRight.set(ControlMode.PercentOutput, -1);
                }
                  
                       }else{
                            Map.intakeLeft.set(ControlMode.PercentOutput, .0);
                    Map.intakeRight.set(ControlMode.PercentOutput, .0);
                       }
// re-zero intake if limit switch is pressed.
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
