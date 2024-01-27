package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

//! This is an intake
public class Intake {
      public static void intakExtension(boolean outIn){
        if(outIn){
            if(){
                Map.movementIntake.set(ControlMode.PercentOutput, 0.8);
            }else{
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
            }
        }else{
            if(){
                Map.movementIntake.set(ControlMode.PercentOutput, -0.8);   
            }else{
                Map.movementIntake.set(ControlMode.PercentOutput, 0);
            }
        }
      }
}