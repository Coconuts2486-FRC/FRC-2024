// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
@SuppressWarnings("removal")
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  static double driverZ;
  static double driverX;
  static double driverY;
  static double time = Timer.getFPGATimestamp();
  

  @Override
  public void robotInit() {
    Misc.isRed();
    Map.swerve.init();
    Launcher.init();
    Intake.init();
    Auto.init();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    Auto.twoPieceStraightFromSpeaker(Misc.getSelectedColor());
  }

  @Override
  public void teleopInit() {
    Map.swerve.init();
    Elevator.init();
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("note", RaspberryPi.gamePieceY());
    SmartDashboard.putNumber("calculated angle", Launcher.regressionForAngle(Misc.getSelectedColor()));
    SmartDashboard.putNumber("encoder angle", Launcher.pivotEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("distance from tag", RaspberryPi.getTagZ4());
    SmartDashboard.putNumber("TagX", RaspberryPi.getTagX4());

  //   if (Map.driver.getRawButton(6) && Map.lightStop.get()) {
  //     Map.backLeft.autoInit(Swerve.blOffset);
  //     Map.backRight.autoInit(Swerve.brOffset);
  //     Map.frontLeft.autoInit(Swerve.flOffset);
  //     Map.frontRight.autoInit(Swerve.frOffset);
  // }
  // // Upon release of button 6 (intake/pickup), reset gyro to the value of gyro2
  // if (Map.driver.getRawButtonReleased(6)) {
  //     Swerve.gyro.setYaw(Swerve.gyro2.getYaw());
  // }
    

    driverZ = Map.driver.getRawAxis(0);
    if (Math.abs(driverZ) < .15) {
      driverZ = 0;
    }
    if (Map.driver.getRawButton(2)) {
      driverZ = Map.driver.getRawAxis(0);
      if (Math.abs(driverZ) < .15) {
        driverZ = 0;
      }

    }

    driverX = Map.driver.getRawAxis(4);
    if (Math.abs(driverX) < .15) {
      driverX = 0;
    }
    if (Map.driver.getRawButton(2)) {
      driverX = -Map.driver.getRawAxis(4);
      if (Math.abs(driverX) < .15) {
        driverX = 0;
      }
    }

    driverY = Map.driver.getRawAxis(5);
    if (Math.abs(driverY) < .15) {
      driverY = 0;
    }
    if (Map.driver.getRawButton(2)) {
      driverY = -Map.driver.getRawAxis(5);
      if (Math.abs(driverY) < .15) {
        driverY = 0;
      }
    }

    Launcher.run(Map.coDriver.getRawButtonPressed(9), Map.coDriver.getRawButton(7),Launcher.manualAngleTuner(Map.coDriver.getPOV()),false,Map.coDriver.getRawButton(3));
    Launcher.launch(Map.coDriver.getRawButton(6));
    Map.swerve.reinit(Map.driver.getRawButton(4));


    if (Map.driver.getRawButton(6) && Map.lightStop.get() == false) {
      RaspberryPi.targetGamePiece(Map.driver.getRawButton(6), Map.driver.getAButtonReleased());
  } 
  // Otherwise, just drive
  else {
      Map.swerve.drive(driverX, driverY,
              RaspberryPi.targetAprilTag(Map.coDriver.getRawButton(5), -driverZ + (driverY * -.001),
                      Misc.getSelectedColor()),false);
  }

    //Launcher.intake(Map.coDriver.getRawButton(5));
    Elevator.run(Map.coDriver.getRawButtonPressed(1), Map.coDriver.getRawButtonPressed(2),Map.coDriver.getRawAxis(3) , Map.coDriver.getRawAxis(2));
    if (Map.driver.getRawButton(6)) {
      Intake.run(
              Map.driver.getRawButton(6),
              Map.coDriver.getRawButtonPressed(4),
              Map.coDriver.getRawButton(6),
              Map.driver.getRawButton(1),
              Map.coDriver.getRawButton(5),
              false,
              Map.driver.getRawAxis(3),
              Map.driver.getRawAxis(2),
              false,
              Misc.getSelectedColor());
  } else {
      Intake.run(
              Map.driver.getRawButton(2),
              Map.coDriver.getRawButtonPressed(4),
              Map.coDriver.getRawButton(6),
              Map.driver.getRawButton(1),
              Map.coDriver.getRawButton(5),
              false,
              Map.driver.getRawAxis(3),
              Map.driver.getRawAxis(2),
              false,
              Misc.getSelectedColor());
  }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    //button 10 is right joystick button
    Elevator.disable(Map.coDriver.getRawButton(10));
    Launcher.disable(Map.coDriver.getRawButton(10));
    Intake.disable(Map.coDriver.getRawButton(10));
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
