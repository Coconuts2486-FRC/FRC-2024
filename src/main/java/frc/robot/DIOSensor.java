package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class represents a Digital Input/Output (DIO) sensor.
 * It provides methods to read the state of the sensor.
 */
public class DIOSensor {

    private static DigitalInput DIO;
    private static String type;

    /**
     * Constructs a DIOSensor object with the specified port and type.
     * 
     * @param port the port number of the DIO sensor
     * @param type the type of the DIO sensor
     */
    public DIOSensor(int port, String type) {
        this.DIO = new DigitalInput(port);
        this.type = type;
    }

    /**
     * Returns the state of the DIO sensor.
     * 
     * @return true if the sensor is active, false otherwise
     */
    public static boolean DIO() {
        if (type == "Srange") {
            if (DIO.get()) {
                return false;
            } else {
                return true;
            }
        } else {
            if (DIO.get()) {
                return true;
            } else {
                return false;
            }
        }
    }
}
