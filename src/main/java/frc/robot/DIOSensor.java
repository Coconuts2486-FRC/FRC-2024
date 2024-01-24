package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DIOSensor {

    public static TalonFX switchTest = new TalonFX(14);
    //hi help

    public static void sensorTest(double speed, double negSpeed) {

        if (limitSwitch.get()) {
            switchTest.set(ControlMode.PercentOutput, 0);
        } else if(lightSensor.get()) {
            switchTest.set(ControlMode.PercentOutput, 0);
        }else{ switchTest.set(ControlMode.PercentOutput, speed - negSpeed);}
    }
}
