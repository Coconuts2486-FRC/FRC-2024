// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.Limelight;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  double startTime;

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {

    // Map.swerve.telemetry();
    // SmartDashboard.putNumber("yaw", Swerve.gyro.getYaw());
    // double rawDistanceFL = Map.frontLeft.driveMotor.getSelectedSensorPosition();
    // SmartDashboard.putNumber("rawDistanceFL", rawDistanceFL);
    // double rawDistanceFR = Map.frontLeft.driveMotor.getSelectedSensorPosition();
    // SmartDashboard.putNumber("rawDistanceFR", rawDistanceFR);
    // double rawDistanceBL = Map.frontLeft.driveMotor.getSelectedSensorPosition();
    // SmartDashboard.putNumber("rawDistanceBL", rawDistanceBL);
    // double rawDistanceBR = Map.frontLeft.driveMotor.getSelectedSensorPosition();
    // SmartDashboard.putNumber("rawDistanceBR", rawDistanceBR);

    // SmartDashboard.putNumber("aprilTagDistance", Limelight.testTagDistance());

  }

  @Override
  public void autonomousInit() {
    // startTime = Auto.getStartTime();
  }

  @Override
  public void autonomousPeriodic() {
    // Auto.runAuto(startTime);
    // Auto.runAutoDrive(7, (0));
  }

  @Override
  public void teleopInit() {
    Map.odometry.init();
    Map.swerve.init();

  }

  @Override
  public void teleopPeriodic() {
    Map.swerve.realignToField(1);
    
    SmartDashboard.putNumber("tagTarget", Limelight.tagTarget(Map.driver.getRawButton(6), Map.driver.getRawAxis(4)));
    SmartDashboard.putNumber("aprilTagDistance", Limelight.testTagDistance());
    // Map.swerve.autoInit();
    // Drive for driver and codriver
    if (  Misc.pov(Map.coDriver.getPOV(),Map.coDriver.getRawButtonPressed(4)) == true){
   
      Map.swerve.drive(Map.coDriver.getRawAxis(4), Map.coDriver.getRawAxis(5),
        Limelight.tagTarget(Map.coDriver.getRawButton(6), Map.coDriver.getRawAxis(0) + (Map.driver.getRawAxis(5)*-.001)));
    } else if (Misc.pov(Map.coDriver.getPOV(),Map.coDriver.getRawButtonPressed(4)) == false){

         Map.swerve.drive(Map.driver.getRawAxis(4), Map.driver.getRawAxis(5),
        Limelight.tagTarget(Map.driver.getRawButton(6), Map.driver.getRawAxis(0) + (Map.driver.getRawAxis(5)*-.001)));
    }
    // Map.swerve.telemetry();

    Map.odometry.calculatePosition();
    if (Map.driver.getRawButton(2)) {
      Map.frontLeft.driveMotor.setSelectedSensorPosition(0);
      Map.frontRight.driveMotor.setSelectedSensorPosition(0);
      Map.backLeft.driveMotor.setSelectedSensorPosition(0);
      Map.backRight.driveMotor.setSelectedSensorPosition(0);

      
    }
    SmartDashboard.putNumber("posX", Map.odometry.calculatePosition()[0]);
    SmartDashboard.putNumber("posY", Map.odometry.calculatePosition()[1]);
    SmartDashboard.putNumber("posXFeet", Map.odometry.calculatePosition()[0] / 42000000);
    SmartDashboard.putNumber("posYFeet", Map.odometry.calculatePosition()[1] / 42000000);

    //Intake
   
    }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Map.swerve.disabled();
    Map.swerve.disabledPos();

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
