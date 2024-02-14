package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.math.controller.PIDController;

//*This is an intake
public class Intake {
        public static PIDController intakePID = new PIDController(.00008,0.00,0.000001);
    public static void init() {
        Map.movementIntake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        Map.intakeRight.setInverted(true);
        Map.movementIntake.setSelectedSensorPosition(0);
        Map.movementIntake.setNeutralMode(NeutralMode.Brake);

    }
    public static void run(boolean button,boolean scoreButton){
        intakExtension(button);
        intakeSpin();
        score(scoreButton);

    }

    public static boolean intakExtension(boolean button) {
        boolean extend = false;
        if (Map.intakeStop.get()){
                Map.movementIntake.setSelectedSensorPosition(0);

        }
        // chang this.
        // only extends at 45 so it can't go past frame perimiter.
        //change this
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
                Map.movementIntake.set(ControlMode.PercentOutput, intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(),99600));
            }
        } else if (extend == false) {
            if (Map.movementIntake.getSupplyCurrent() > 5) {
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
            } // change this to out position
            else {
                //45 change this
                if (Map.launcherPivot.getSelectedSensorPosition()<-21000){
                Map.movementIntake.set(ControlMode.PercentOutput, intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(),36400));
                }else if(Map.launcherPivot.getSelectedSensorPosition()>-21000){
                       Map.movementIntake.set(ControlMode.PercentOutput, intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(),36400));

                }
            }
        }
        return extend;
    }

    // bottom function makes it so when a button is pressed it activates using the
    // top function

    // bottom function is what is used to spin up the wheels for intaking in notes
    public static void intakeSpin() {

    
        if (Map.movementIntake.getSelectedSensorPosition()>99000) {
            Map.intakeRight.set(ControlMode.PercentOutput, .5);
            Map.intakeLeft.set(ControlMode.PercentOutput, .5);
            if (Map.lightStop.get()){
                     Map.intakeRight.set(ControlMode.PercentOutput, 0);
              Map.intakeLeft.set(ControlMode.PercentOutput, 0);
            }
        } else {
            Map.intakeRight.set(ControlMode.PercentOutput, .0);
            Map.intakeLeft.set(ControlMode.PercentOutput, .0);
        }
    }

    public static void test(boolean toggle1, boolean toggle2){
        boolean toggleOut = false;
        boolean toggleScore = false;
        if (toggle2){
            toggleScore = !toggleScore;
        }
        if (toggle1){
            toggleOut = !toggleOut;
        }

    
        if (toggle1 == true) {
            Map.movementIntake.set(ControlMode.PercentOutput,intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 98000));
          
        }

        if (toggleOut == false) {
            if (toggleScore){
                  Map.movementIntake.set(ControlMode.PercentOutput, intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 30000));
            }
            else if (toggleScore==false){
            Map.movementIntake.set(ControlMode.PercentOutput, intakePID.calculate(Map.movementIntake.getSelectedSensorPosition(), 0));

            if (Map.intakeStop.get()) {
                Map.movementIntake.setSelectedSensorPosition(0);
            }
        }
    }
    }

    



    // bottom function is what is used to score on the amp
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
