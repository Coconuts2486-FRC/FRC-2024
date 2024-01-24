package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class limitSwitch {
    public static DigitalInput limitSwitch = new DigitalInput(0);
    
    public static TalonFX switchTest = new TalonFX(14);
    //hi help

    public static void limitTest(double speed, double negSpeed) {
        switchTest.setNeutralMode(NeutralMode.Brake);
        if (limitSwitch.get()) {
            switchTest.set(ControlMode.PercentOutput, 0);
        } else {
            switchTest.set(ControlMode.PercentOutput, speed - negSpeed);
        }
    }
}
