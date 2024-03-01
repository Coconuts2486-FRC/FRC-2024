package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.AutoPaths;

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


  public static void selectAuto(){
    Map.selectedAuto.setDefaultOption("Two note, straight from speaker", 1);
    Map.selectedAuto.addOption("Three piece from amp side, don't leave zone", 2);
    Map.selectedAuto.addOption("Drive out from zone", 3);
    SmartDashboard.putData("SelectAuto", Map.selectedAuto);

  }


 public static int getSelectedAuto() {
    return (int) Map.selectedAuto.getSelected();
  }

  public static void runSelectedAuto(boolean red){
    if (getSelectedAuto() == 1){
      AutoPaths.twoPieceStraightFromSpeaker(red);
    }else if(getSelectedAuto() == 2){
      AutoPaths.threePieceAmpSideStayInZone(red);
    }else if (getSelectedAuto() == 3){
      AutoPaths.backUp(red);
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
   * @param pov the POV button
   * @param button the state of the button
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
