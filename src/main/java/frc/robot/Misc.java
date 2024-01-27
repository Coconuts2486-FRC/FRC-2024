package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Misc {
    public static boolean switched = false;

    public static boolean pov(int pov, boolean button) {
        SmartDashboard.putNumber("POV", Map.driver.getPOV());
        if (pov == 0) {
            if (button) {
                switched = !switched;
            }
        }
        return switched;
    }
}
