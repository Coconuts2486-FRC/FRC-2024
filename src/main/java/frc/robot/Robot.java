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
    public static double autoStart;

    @Override
    public void robotInit() {
        Misc.isRed();
        Misc.selectAuto();
        Map.swerve.init();
        Launcher.init();
        Intake.init();
        Auto.init();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putBoolean("elevator Bottom DIO", Map.elevatorBottom.get());
        SmartDashboard.putBoolean("LightStop", Map.lightStop.get());
        SmartDashboard.putNumber("extend", Map.intakeExtend.getSelectedSensorPosition());
      SmartDashboard.putNumber("gyro", Swerve.gyro.getYaw());
        SmartDashboard.putNumber("Yval", Map.odometry.calculatePosition()[1]);
        SmartDashboard.putNumber("note", RaspberryPi.gamePieceZ());
        SmartDashboard.putNumber("calculated angle", Launcher.regressionForAngle(Misc.getSelectedColor()));
        SmartDashboard.putNumber("encoder angle", Launcher.pivotEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("distance from tag 4", RaspberryPi.getTagZ4());
          SmartDashboard.putNumber("distance from tag 7", RaspberryPi.getTagZ4());
        SmartDashboard.putNumber("Tag4X", RaspberryPi.getTagX4());
                SmartDashboard.putNumber("Tag7X", RaspberryPi.getTagX7());
        SmartDashboard.putNumber("noteAngle", RaspberryPi.gamePieceAngle());
        SmartDashboard.putNumber("noteX", RaspberryPi.gamePieceX());

    }

    @Override
    public void autonomousInit() {

        Auto.init();
        autoStart = Timer.getFPGATimestamp();
    }

    @Override
    public void autonomousPeriodic() {
      SmartDashboard.putNumber("rightLaunchAuto", Map.rightLauncher.getSelectedSensorVelocity());
        SmartDashboard.putNumber("a", Auto.a);
        // Auto.threePieceCenterLine(Misc.getSelectedColor());
       Misc.runSelectedAuto(Misc.getSelectedColor());
        //Auto.fourPieceStraightFromSpeaker(Misc.getSelectedColor());
    }

    @Override
    public void teleopInit() {
        Map.swerve.init();
        Elevator.init();
        Map.odometry.init();
    }

    @Override
    public void teleopPeriodic() {

        // if (Map.driver.getRawButton(6) && Map.lightStop.get()) {
        // Map.backLeft.autoInit(Swerve.blOffset);
        // Map.backRight.autoInit(Swerve.brOffset);
        // Map.frontLeft.autoInit(Swerve.flOffset);
        // Map.frontRight.autoInit(Swerve.frOffset);
        // }
        // // Upon release of button 6 (intake/pickup), reset gyro to the value of gyro2
        // if (Map.driver.getRawButtonReleased(6)) {
        // Swerve.gyro.setYaw(Swerve.gyro2.getYaw());
        // }

        driverZ = Map.driver.getRawAxis(0) - (Math.signum(driverZ)*.05);
        if (Math.abs(Map.driver.getRawAxis(0)) < .15) {
            driverZ = 0;
        }
        

        driverX = Map.driver.getRawAxis(4);
        if (Math.abs(driverX) < .15) {
            driverX = 0;
        }
    

        driverY = Map.driver.getRawAxis(5);
        if (Math.abs(driverY) < .15) {
            driverY = 0;
        }

        Launcher.run(Map.coDriver.getRawButtonPressed(9), Map.coDriver.getRawButton(7),
                Launcher.manualAngleTuner(Map.coDriver.getPOV()), false, Map.coDriver.getRawButton(3),Map.driver.getRawButton(8));
        Launcher.launch(Map.coDriver.getRawButton(6));
        Map.swerve.reinit(Map.driver.getRawButton(4));

        if (Map.driver.getRawButton(6) && Map.lightStop.get() == false) {
            RaspberryPi.targetGamePiece(Map.driver.getRawButton(6), Map.driver.getAButtonReleased());
        }
        // Otherwise, just drive
        else {
            Map.swerve.drive(driverX, driverY,
                    RaspberryPi.targetAprilTag(Map.coDriver.getRawButton(5),-driverZ ,
                            Misc.getSelectedColor()),
                    false);
        }

        // Launcher.intake(Map.coDriver.getRawButton(5));
         Elevator.run(Map.coDriver.getRawButtonPressed(1), Map.coDriver.getRawButtonPressed(2),
        //         Map.coDriver.getRawAxis(3), Map.coDriver.getRawAxis(2),Map.driver.getRawButton(7));
       // Elevator.manualRun(Map.coDriver.getRawButton(1), Map.coDriver.getRawButton(2),
                 Map.coDriver.getRawAxis(3), Map.coDriver.getRawAxis(2),Map.driver.getRawButton(7));
        if (Map.driver.getRawButton(6)) {
            Intake.auto(
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
            Intake.auto(
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
                
        if (Map.driver.getRawButton(8)){
            Launcher.init();
  
        }
        if (Map.driver.getRawButton(7)){
            Elevator.init();
  
        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        // button 10 is right joystick button
        Elevator.disable(Map.coDriver.getRawButton(10));
        Launcher.disable(Map.coDriver.getRawButton(10));
        Intake.disable(Map.coDriver.getRawButton(10));
        Map.swerve.disabled(Map.coDriver.getRawButton(10));
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
