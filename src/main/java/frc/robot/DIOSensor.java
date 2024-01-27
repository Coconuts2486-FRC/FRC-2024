package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DIOSensor {
    private static DigitalInput DIO;

    public DIOSensor(int port, String type) {
        this.DIO = new DigitalInput(port);
    }

    public static boolean DIO() {

        if (DIO.get()) {
            return true;
        } else {
            return false;
        }
    }
}
