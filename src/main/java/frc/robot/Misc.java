package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Misc {
    public static boolean switched = false;
    
   public static void isRed (){
        Map.red.setDefaultOption("Red", true);
        Map.red.addOption("Blue", false);
        SmartDashboard.putData("Alliance", Map.red);
    }
    public static boolean getSelectedColor() {
        return (boolean) Map.red.getSelected();
    }

    public static void putColor(){
        if(getSelectedColor()){
            SmartDashboard.putString("Color", "Red");
        }else{
            SmartDashboard.putString("Color", "Blue");
        }
    }

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
