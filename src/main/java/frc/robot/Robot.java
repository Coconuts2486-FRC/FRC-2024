// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision.Limelight;
import frc.robot.Vision.RaspberryPi;

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
  boolean toggleDown = true;
  boolean toggleUp = false;
  public static SendableChooser red = new SendableChooser<>();

  @Override
  public void robotInit() {
    Launcher.init();
    Misc.isRed();
    Launcher.init();
    Misc.switched = false;
    Elevator.init();
    
  }

  @Override
  public void robotPeriodic() {
     SmartDashboard.putNumber("elevator",Map.rightElevator.getSelectedSensorPosition());
    Misc.putColor();
    SmartDashboard.putBoolean("DIOFORELEVATORB",    Map.elevatorBottom.get());
    SmartDashboard.putBoolean("DIOFORELEVATORT",    Map.elevatorTop.get());
    SmartDashboard.putBoolean("LIGHTSTOP",  Map.lightStop.get());
     SmartDashboard.putBoolean("DIOFORPIVOT",    Map.pivotTop.get());
    SmartDashboard.putBoolean("DIOFORINTAKE",    Map.intakeStop.get());


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
    SmartDashboard.putNumber("ringZ", RaspberryPi.gamePieceZ());
     SmartDashboard.putNumber("ringX", RaspberryPi.gamePieceX());

  }

  @Override
  public void teleopInit() {
    Map.odometry.init();
    Map.swerve.init();
      Launcher.init();
      toggleDown = true;
   toggleUp = false;


  }

  @Override
  public void teleopPeriodic() {
   
    

    
    // Map.swerve.autoInit();
    
    
    // CoDriver code
    
    if (  Misc.pov(Map.coDriver.getPOV(),Map.coDriver.getRawButtonPressed(8)) == true){
          Map.swerve.realignToField(Map.coDriver.getRawButton(1));
      if (Map.coDriver.getRawAxis(2)>.2 ){
         Map.swerve.drive(0,RaspberryPi.driveToGamePiece(),RaspberryPi.targetGamePiece());

      }
      else{
      Map.swerve.drive(Map.coDriver.getRawAxis(4), Map.coDriver.getRawAxis(5),
        RaspberryPi.targetAprilTag(Map.coDriver.getRawButton(6), Map.coDriver.getRawAxis(0) + (Map.driver.getRawAxis(5)*-.001),Misc.getSelectedColor()));
      }
        Elevator.test(Map.driver.getRawButtonPressed(1),Map.driver.getRawAxis(5));
       // Elevator.run(Map.driver.getRawButtonPressed(1), Map.driver.getRawButtonPressed(2), Map.driver.getRawButtonPressed(3), Map.driver.getRawButtonPressed(4), Map.driver.getRawAxis(2) );
    //Driver code
    
      } else if (Misc.pov(Map.coDriver.getPOV(),Map.coDriver.getRawButtonPressed(8)) == false){
             Map.swerve.realignToField(Map.driver.getRawButton(1));
        if (Map.driver.getRawAxis(2)>.2 ){
           Map.swerve.drive(0,RaspberryPi.driveToGamePiece(),RaspberryPi.targetGamePiece());


         } else{
         Map.swerve.drive(Map.driver.getRawAxis(4), Map.driver.getRawAxis(5),
        RaspberryPi.targetAprilTag(Map.driver.getRawButton(6), Map.driver.getRawAxis(0) + (Map.driver.getRawAxis(5)*-.001),Misc.getSelectedColor()));
    }


  

    //.Elevator.run(Map.coDriver.getRawButtonPressed(1), Map.coDriver.getRawButtonPressed(2), Map.coDriver.getRawButtonPressed(3), Map.coDriver.getRawButtonPressed(4), Map.coDriver.getRawAxis(1));
    Elevator.test(Map.coDriver.getRawButtonPressed(1),Map.coDriver.getRawAxis(5));
   


  }


    
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
