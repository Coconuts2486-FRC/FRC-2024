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
import frc.robot.Auto.AutoPaths;
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
  public static double saveYaw;
  double startTime;
  public static double autoStart;
  static double teleopStart;
public static double teleopTime;
  static double driverZ;
  static double driverX;
  static double driverY;
  static double zeroTime;
  public static SendableChooser red = new SendableChooser<>();
   public static boolean launchReady;
   public static PIDController  zeroPID= new PIDController(.01,0,0) ;
  @Override
  public void robotInit() {
 
    Launcher.init();
    Intake.init();
    Misc.isRed();
    Misc.selectAuto();
    Misc.switched = false;
    Elevator.init();

    Map.odometry.init();
    Map.swerve.init();
  }

  @Override
  public void robotPeriodic() {

    
    SmartDashboard.putBoolean("LIGHTSTOP",  Map.lightStop.get());
     SmartDashboard.putBoolean("DIOFORPIVOT",    Map.pivotTop.get());
     Launcher.manualAngleTuiner(Map.coDriver.getPOV());
   


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
    AutoPaths.autoInit();
        Launcher.init();
    Intake.init();
    
   // Map.odometry.init();
   // Map.swerve.init();
    
    // startTime = Auto.getStartTime();
  }

  @Override
  public void autonomousPeriodic() {
   // Map.leftLauncher.set(ControlMode.Velocity, 21000)  ;

  
//   AutoPaths.auto2();
    Misc.runSelectedAuto(Misc.getSelectedColor());

    // AutoPaths.auto2(Misc.getSelectedColor());
    // AutoPaths.twoPieceStraightFromSpeaker(Misc.getSelectedColor());
   
  }

  @Override
  public void teleopInit() {
       teleopStart = Timer.getFPGATimestamp();

    Map.odometry.init();
    Map.swerve.init();
    Elevator.init();
  //  Launcher.init();
   
    

  }

  @Override
  public void teleopPeriodic() {

     teleopTime = Timer.getFPGATimestamp();
    if(Map.driver.getRawButton(6)&& Map.lightStop.get()){
    Map.backLeft.autoInit(Swerve.blOffset);
       Map.backRight.autoInit(Swerve.brOffset);
          Map.frontLeft.autoInit(Swerve.flOffset);
             Map.frontRight.autoInit(Swerve.frOffset);
     } if(Map.driver.getRawButtonReleased(6)){
     Swerve.gyro.setYaw(Swerve.gyro2.getYaw() );
     }

    driverZ = Map.driver.getRawAxis(0) ;
     if (Math.abs(driverZ)<.15){
      driverZ = 0;  
    }
    if(Map.driver.getRawButton(2)){
         driverZ = -Map.driver.getRawAxis(0) ;
          if (Math.abs(driverZ)<.15){
      driverZ = 0;  
      }
   
    }
    
     driverX = Map.driver.getRawAxis(4) ;
    if (Math.abs(driverX)<.15){
      driverX = 0;
    }
     if(Map.driver.getRawButton(2)){
         driverX = -Map.driver.getRawAxis(4) ;
          if (Math.abs(driverX)<.15){
      driverX = 0;  
      }
    }

     driverY = Map.driver.getRawAxis(5) ;
    if (Math.abs(driverY)<.15){
      driverY = 0;
    }
     if(Map.driver.getRawButton(2)){
         driverY = -Map.driver.getRawAxis(5) ;
          if (Math.abs(driverY)<.15){
      driverY = 0;  
      }
    }

    
 
    

    // Map.swerve.autoInit();

    // CoDriver code
    
    // if (  Misc.pov(Map.coDriver.getPOV(),Map.coDriver.getRawButtonPressed(8)) == true){
    //       Map.swerve.realignToField(Map.coDriver.getRawButton(1));
    //   // if (Map.coDriver.getRawAxis(2)>.7 ){
    //   //    Map.swerve.drive(0,RaspberryPi.driveToGamePiece(),RaspberryPi.targetGamePiece());

    //   // }
    //   //else{
    //   Map.swerve.drive(Map.coDriver.getRawAxis(4), Map.coDriver.getRawAxis(5),
    //     RaspberryPi.targetAprilTag(Map.coDriver.getRawButton(8),
    //      -Map.coDriver.getRawAxis(0) + (Map.driver.getRawAxis(5)*-.001),Misc.getSelectedColor()));
    // //  }
    //     Elevator.test(Map.driver.getRawButtonPressed(1),Map.driver.getRawAxis(5));
    //       Intake.test(Map.coDriver.getRawButtonPressed(5),Map.driver.getRawButtonPressed(5),Map.driver.getRawButton(9));
         //   Launcher.test(Map.driver.getRawButton(6), Misc.getSelectedColor());
           // Intake.intakeSpin();
       // Elevator.run(Map.driver.getRawButtonPressed(1), Map.driver.getRawButtonPressed(2), Map.driver.getRawButtonPressed(3), Map.driver.getRawButtonPressed(4), Map.driver.getRawAxis(2) );
    //Driver code
    
      // } else if (Misc.pov(Map.coDriver.getPOV(),Map.coDriver.getRawButtonPressed(8)) == false){

        //  ^
        //  | un comment for codrive code
       // if (Map.)

             Map.swerve.reinit(Map.driver.getRawButton(4));


        if (Map.driver.getRawButton(6) && Map.lightStop.get()== false ){
      RaspberryPi.targetGamePiece(Map.driver.getRawButton(6), Map.driver.getAButtonReleased());
         } else{
         Map.swerve.drive(driverX, driverY,
        RaspberryPi.targetAprilTag(Map.coDriver.getRawButton(5), -driverZ + (driverY*-.001),Misc.getSelectedColor()));
    }

// launcher wheel code

    Launcher.launch(Map.coDriver.getRawButton(6));
  

    //.Elevator.run(Map.coDriver.getRawButtonPressed(1), Map.coDriver.getRawButtonPressed(2), Map.coDriver.getRawButtonPressed(3), Map.coDriver.getRawButtonPressed(4), Map.coDriver.getRawAxis(1));
    Elevator.test(Map.coDriver.getRawButtonPressed(1),Map.coDriver.getRawAxis(5),Map.coDriver.getRawButtonPressed(2),Map.coDriver.getRawButtonPressed(3));

if (Map.driver.getRawButton(6)){
   Intake.test(Map.driver.getRawButton(6),Map.coDriver.getRawButtonPressed(4),Map.coDriver.getRawButton(6),Map.driver.getRawButton(1),Map.coDriver.getRawButton(5),false,Map.driver.getRawAxis(3),Map.driver.getRawAxis(2),false,Misc.getSelectedColor());
}else{
   Intake.test(Map.driver.getRawButton(2),Map.coDriver.getRawButtonPressed(4),Map.coDriver.getRawButton(6),Map.driver.getRawButton(1),Map.coDriver.getRawButton(5),false,Map.driver.getRawAxis(3),Map.driver.getRawAxis(2),false,Misc.getSelectedColor());
}
   //Intake.test(Map.driver.getRawButton(6),Map.coDriver.getRawButtonPressed(4),Map.coDriver.getRawButton(6),Map.coDriver.getRawButton(7),false);
    SmartDashboard.putNumber("launcher Position", Map.launcherPivot.getSelectedSensorPosition());
     Launcher.test(Map.driver.getRawButton(3),Map.coDriver.getRawButtonPressed(3), Map.driver.getRawButton(1), Map.coDriver.getRawButton(5),Map.coDriver.getRawAxis(5),Misc.getSelectedColor(),true,false,Launcher.manualAngleTuiner(Map.coDriver.getPOV()));
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


    
  //}

  @Override
  public void disabledInit() {
   
  }

  @Override
  public void disabledPeriodic() {
    Map.swerve.disabled();
    //Map.swerve.disabledPos();
     Launcher.disable(Map.coDriver.getRawButton(10));
     Intake.disable(Map.coDriver.getRawButton(10));
         
     Elevator.disable(Map.coDriver.getRawButton(10));
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
