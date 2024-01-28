package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

//*This is an intake
public class Intake {
    static boolean extend = false;
      public static void intakExtension(boolean outIn){
        if(outIn){
            if(Map.intakeExtendStop.DIO()){
                Map.movementIntake.set(ControlMode.PercentOutput, 0.8);
            }else{
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
            }
        }else{
            if(Map.intakeCloseStop.DIO()){
                Map.movementIntake.set(ControlMode.PercentOutput, -0.8);   
            }else{
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
            }
        }
      }
//bottom function makes it so when a button is pressed it activates using the top function
      public static void buttonExtend(boolean button){
        if(button){
            extend= !extend;
        }
        if(Map.intakeStop.DIO()){
            extend= false;
        }
        intakExtension(extend);
        intakeSpin(extend);           
    }
//bottom function is what is used to spin up the wheels for intaking in notes
   public static void intakeSpin(boolean buttonOne){
    if(buttonOne){
        Map.intakeRight.set(ControlMode.PercentOutput, 0.8);
        Map.intakeLeft.set(ControlMode.PercentOutput, -0.8);
    }else{
        Map.intakeRight.set(ControlMode.PercentOutput, 0);
        Map.intakeLeft.set(ControlMode.PercentOutput, 0);
        }
    }  
//bottom function is what is used to score on the amp
   public static void ampOutake(boolean outakeAmp){
    if(outakeAmp){
        Map.intakeRight.set(ControlMode.PercentOutput, -0.8);
        Map.intakeLeft.set(ControlMode.PercentOutput, 0.8);
    }else{
        Map.intakeRight.set(ControlMode.PercentOutput, 0);
        Map.intakeLeft.set(ControlMode.PercentOutput, 0);
    }
   }
    
} 
