package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Auto.AutoPaths;

/**
 * This class contains miscellaneous methods that are used throughout the robot
 * code.
 */
public class Misc {

    public static boolean switched = false;

    /**
     * Sets the alliance color to red.
     */
    public static void isRed() {
        Map.red.setDefaultOption("Red", true);
        Map.red.addOption("Blue", false);
        SmartDashboard.putData("Alliance", Map.red);
    }

    /**
     * Select which AUTO path to run
     */
    public static void selectAuto() {
        Map.selectedAuto.setDefaultOption("The Three Note that goes out", 1);
        Map.selectedAuto.addOption("Four piece from subwoofer", 2);
              Map.selectedAuto.addOption("the old out one", 3);
              Map.selectedAuto.addOption("The close sweep", 4);
        SmartDashboard.putData("SelectAuto", Map.selectedAuto);

    }

    /**
     * Return the selected AUTO path
     * 
     * @return int
     */
    public static int getSelectedAuto() {
        return (int) Map.selectedAuto.getSelected();
    }

    /**
     * Run the selected AUTO path
     * 
     * @param red Are we red alliance?
     */
    public static void runSelectedAuto(boolean red) {
        if (getSelectedAuto() == 1) {
            Auto.threeNoteTwoOut(red);
        } else if (getSelectedAuto() == 2) {
            Auto.fourPieceStraightFromSpeaker(red);
    } else if ( getSelectedAuto() == 3){
           Auto.threePieceCenterLine(red);
    } else if ( getSelectedAuto() == 4){
        Auto.closeSweep(red);
    }
    }
    /**
     * Returns the selected alliance color.
     *
     * @return true if the alliance color is red, false otherwise
     */
    public static boolean getSelectedColor() {
        return (boolean) Map.red.getSelected();
    }

    /**
     * Puts the selected alliance color on the SmartDashboard.
     */
    public static void putColor() {
        if (getSelectedColor()) {
            SmartDashboard.putString("Color", "Red");
        } else {
            SmartDashboard.putString("Color", "Blue");
        }
    }

    /**
     * Returns the state of the POV button.
     *
     * @param pov    The POV button
     * @param button The state of the button
     * @return true if the button is pressed, false otherwise
     */
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
