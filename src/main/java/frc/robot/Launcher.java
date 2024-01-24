package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Launcher {
    public static void powerUp(){
        Map.rightLauncher.set(ControlMode.PercentOutput, 1);
        Map.leftLauncher.set(ControlMode.PercentOutput, 1);
    }
    public static void aim(boolean leftTrigger){
      powerUp();
      //hi
    }
}
