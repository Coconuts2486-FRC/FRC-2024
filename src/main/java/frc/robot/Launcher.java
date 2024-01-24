package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Launcher {
    public static void powerUp(boolean buttonPress){
        if(buttonPress){
            Map.rightLauncher.set(ControlMode.PercentOutput, 1);
            Map.leftLauncher.set(ControlMode.PercentOutput, 1); 
        }else{
            Map.rightLauncher.set(ControlMode.PercentOutput, 0);
            Map.leftLauncher.set(ControlMode.PercentOutput, 0);
        }
       
    }
    public static void aim(boolean leftTrigger,int pivotPosistion){
        if(leftTrigger){
            powerUp(leftTrigger);
            Map.launcherPivot.set(ControlMode.Position, pivotPosistion);
        }
      
     
    }
}
