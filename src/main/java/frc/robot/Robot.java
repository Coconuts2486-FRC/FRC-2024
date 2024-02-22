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
  static double teleopStart;
public static double teleopTime;
  static double driverZ;
  static double driverX;
  static double driverY;
  public static SendableChooser red = new SendableChooser<>();
   public static boolean launchReady;
  @Override
  public void robotInit() {
 
    Launcher.init();
    Intake.init();
    Misc.isRed();
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
       SmartDashboard.putNumber("intakeValues",Map.movementIntake.getSelectedSensorPosition());


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
      double time = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("timer", time);
    // Auto.runAuto(startTime);
    // Auto.runAutoDrive(7, (0));
    SmartDashboard.putNumber("ringZ", RaspberryPi.gamePieceY());
    SmartDashboard.putNumber("ringX", RaspberryPi.gamePieceX());
  }

  @Override
  public void teleopInit() {
       teleopStart = Timer.getFPGATimestamp();

    Map.odometry.init();
    Map.swerve.init();
    Elevator.init();
    Launcher.init();
    Intake.init();
    

  }

  @Override
  public void teleopPeriodic() {

     teleopTime = Timer.getFPGATimestamp();
   

    driverZ = Map.driver.getRawAxis(0) ;
    if (Math.abs(driverZ)<.15){
      driverZ = 0;
    }


     driverX = Map.driver.getRawAxis(4) ;
    if (Math.abs(driverX)<.15){
      driverX = 0;
    }

     driverY = Map.driver.getRawAxis(5) ;
    if (Math.abs(driverY)<.15){
      driverY = 0;
    }


    SmartDashboard.putNumber("X", driverX);
    
    SmartDashboard.putNumber("teleopTime", teleopTime-teleopStart);
    SmartDashboard.putNumber("RegPos", Map.launcherPivot.getSelectedSensorPosition());
    SmartDashboard.putNumber("pieceX",RaspberryPi.gamePieceX());
        SmartDashboard.putNumber("pieceZ",RaspberryPi.gamePieceY());
       SmartDashboard.putNumber("launcherR",Map.rightLauncher.getSelectedSensorVelocity());
        SmartDashboard.putNumber("launcherL",Map.leftLauncher.getSelectedSensorVelocity());
if (Map.rightLauncher.getSelectedSensorVelocity()<-18000&&Map.leftLauncher.getSelectedSensorVelocity()>18000){
  launchReady = true;
}else{
  launchReady = false;
}
SmartDashboard.putBoolean("launchReady", launchReady);

    // Map.swerve.autoInit();

    // CoDriver code
    
    if (  Misc.pov(Map.coDriver.getPOV(),Map.coDriver.getRawButtonPressed(8)) == true){
          Map.swerve.realignToField(Map.coDriver.getRawButton(1));
      // if (Map.coDriver.getRawAxis(2)>.7 ){
      //    Map.swerve.drive(0,RaspberryPi.driveToGamePiece(),RaspberryPi.targetGamePiece());

      // }
      //else{
      Map.swerve.drive(Map.coDriver.getRawAxis(4), Map.coDriver.getRawAxis(5),
        RaspberryPi.targetAprilTag(Map.coDriver.getRawButton(8),
         -Map.coDriver.getRawAxis(0) + (Map.driver.getRawAxis(5)*-.001),Misc.getSelectedColor()));
    //  }
        Elevator.test(Map.driver.getRawButtonPressed(1),Map.driver.getRawAxis(5));
          Intake.test(Map.coDriver.getRawButtonPressed(5),Map.driver.getRawButtonPressed(5),Map.driver.getRawButton(9));
         //   Launcher.test(Map.driver.getRawButton(6), Misc.getSelectedColor());
           // Intake.intakeSpin();
       // Elevator.run(Map.driver.getRawButtonPressed(1), Map.driver.getRawButtonPressed(2), Map.driver.getRawButtonPressed(3), Map.driver.getRawButtonPressed(4), Map.driver.getRawAxis(2) );
    //Driver code
    
      } else if (Misc.pov(Map.coDriver.getPOV(),Map.coDriver.getRawButtonPressed(8)) == false){
             Map.swerve.realignToField(Map.driver.getRawButton(1));
        if (Map.driver.getRawButton(5) ){
      RaspberryPi.targetGamePiece(Map.driver.getRawButtonPressed(6),Map.driver.getRawButton(6),Map.driver.getRawButtonReleased(6));
         } else{
         Map.swerve.drive(driverX, driverY,
        RaspberryPi.targetAprilTag(Map.driver.getRawButton(8), -driverZ + (driverY*-.001),Misc.getSelectedColor()));
    }

// launcher wheel code
  if(Map.coDriver.getRawButton(2)){
    Map.leftLauncher.set(ControlMode.Velocity, 21000);
    Map.rightLauncher.set(ControlMode.Velocity, 21000);
    SmartDashboard.putNumber("rightLaunch", Map.rightLauncher.getSelectedSensorVelocity());
        SmartDashboard.putNumber("leftLaunch", Map.leftLauncher.getSelectedSensorVelocity());
  }else{
     Map.leftLauncher.set(ControlMode.PercentOutput, .0);
    Map.rightLauncher.set(ControlMode.PercentOutput, -.0);
  }

    //.Elevator.run(Map.coDriver.getRawButtonPressed(1), Map.coDriver.getRawButtonPressed(2), Map.coDriver.getRawButtonPressed(3), Map.coDriver.getRawButtonPressed(4), Map.coDriver.getRawAxis(1));
    Elevator.test(Map.coDriver.getRawButtonPressed(1),Map.coDriver.getRawAxis(5));
    SmartDashboard.putNumber("intake", Map.movementIntake.getSelectedSensorPosition());

   Intake.test(Map.driver.getRawButton(6),Map.coDriver.getRawButtonPressed(5),Map.coDriver.getRawButtonPressed(6));
    SmartDashboard.putNumber("launcher", Map.launcherPivot.getSelectedSensorPosition());
     Launcher.test(Map.coDriver.getRawButton(8),Map.coDriver.getRawButtonPressed(3), Map.coDriver.getRawButton(7),Map.coDriver.getRawAxis(5),Misc.getSelectedColor());
//  if (teleopTime-teleopStart<.4){
//          Launcher.test(true, false, false, 0, Misc.getSelectedColor());
//     // }else if (teleopTime-teleopStart >=.7&& teleopTime-teleopStart < 1.3){
//     //       Launcher.test(true,false,false,0,Misc.getSelectedColor());

//      } else if(teleopTime-teleopStart>.4&&teleopTime-teleopStart<.7){
//        Launcher.test(Map.coDriver.getRawButton(8),true, Map.coDriver.getRawButton(7),Map.coDriver.getRawAxis(5),Misc.getSelectedColor());
//     }else if (teleopTime-teleopStart>.7&&teleopTime-teleopStart<1){
//         Launcher.test(true, false, false, 0, Misc.getSelectedColor());
//     }else{
//        Launcher.test(Map.coDriver.getRawButton(8),true, Map.coDriver.getRawButton(7),Map.coDriver.getRawAxis(5),Misc.getSelectedColor());
//     }



    
   
    
    //Intake.intakeSpin();


  }


    
  }

  @Override
  public void disabledInit() {
   
  }

  @Override
  public void disabledPeriodic() {
    Map.swerve.disabled();
    Map.swerve.disabledPos();
     Launcher.disable(Map.coDriver.getRawButton(2));
     Intake.disable(Map.coDriver.getRawButton(2));
         
     Elevator.disable(Map.coDriver.getRawButton(2));
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
