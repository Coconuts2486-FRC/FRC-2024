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
    public static PIDController zeroPID = new PIDController(.01, 0, 0);

    /**
     * Robot-wide initialization code (overrides IterativeRobotBase.robotInit)
     */
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

    /**
     * Periodic code for all robot modes (overrides
     * IterativeRobotBase.robotPeriodic)
     */
    @Override
    public void robotPeriodic() {
        SmartDashboard.putBoolean("bottomLimit", Map.elevatorBottom.get());
        SmartDashboard.putNumber("elevator", Map.leftElevator.getSelectedSensorPosition());
        SmartDashboard.putNumber("rlencoder", Map.intakeRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("a", AutoPaths.a);

        SmartDashboard.putBoolean("LIGHTSTOP", Map.lightStop.get());
        SmartDashboard.putBoolean("DIOFORPIVOT", Map.pivotTop.get());
        Launcher.manualAngleTuner(Map.coDriver.getPOV());

    }

    /**
     * Initialization code for autonomous mode (overrides
     * IterativeRobotBase.autonomousInit)
     */
    @Override
    public void autonomousInit() {
        startTime = Timer.getFPGATimestamp();
        AutoPaths.autoInit();
        Launcher.init();
        Intake.init();

        // Map.odometry.init();
        // Map.swerve.init();

        // startTime = Auto.getStartTime();
    }

    /**
     * Periodic code for autonomous mode (overrides
     * IterativeRobotBase.autonomousPeriodic)
     */
    @Override
    public void autonomousPeriodic() {
        // Map.leftLauncher.set(ControlMode.Velocity, 21000) ;
        // uncomment for drive foreward
        // if (Timer.getFPGATimestamp()-startTime<3) {
        // Map.swerve.drive(0, -.25, 0);
        // }else
        // if(Timer.getFPGATimestamp()-startTime>3){
        // Map.swerve.drive(0, 0, 0);
        // }
        // <->

        // AutoPaths.auto2();
        Misc.runSelectedAuto(Misc.getSelectedColor());
        // AutoPaths.auto2(Misc.getSelectedColor());
        // AutoPaths.twoPieceStraightFromSpeaker(Misc.getSelectedColor());

    }

    /**
     * Exit code for autonomous mode (overrides IterativeRobotBase.autonomousExit)
     */
    @Override
    public void autonomousExit() {
    }

    /**
     * Initialization code for teleop mode (overrides IterativeRobotBase.teleopInit)
     */
    @Override
    public void teleopInit() {
        teleopStart = Timer.getFPGATimestamp();

        Map.odometry.init();
        Map.swerve.init();
        Elevator.init();
        // Launcher.init();
    }

    /**
     * Periodic code for teleop mode (overrides IterativeRobotBase.teleopPeriodic)
     */
    @Override
    public void teleopPeriodic() {

        SmartDashboard.putBoolean("45true", Launcher.goTo45);
        teleopTime = Timer.getFPGATimestamp();
        if (Map.driver.getRawButton(6) && Map.lightStop.get()) {
            Map.backLeft.autoInit(Swerve.blOffset);
            Map.backRight.autoInit(Swerve.brOffset);
            Map.frontLeft.autoInit(Swerve.flOffset);
            Map.frontRight.autoInit(Swerve.frOffset);
        }
        if (Map.driver.getRawButtonReleased(6)) {
            Swerve.gyro.setYaw(Swerve.gyro2.getYaw());
        }

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

        Map.swerve.reinit(Map.driver.getRawButton(4));

        if (Map.driver.getRawButton(6) && Map.lightStop.get() == false) {
            RaspberryPi.targetGamePiece(Map.driver.getRawButton(6), Map.driver.getAButtonReleased());
        } else {
            Map.swerve.drive(driverX, driverY,
                    RaspberryPi.targetAprilTag(Map.coDriver.getRawButton(5), -driverZ + (driverY * -.001),
                            Misc.getSelectedColor()));
        }

        // launcher wheel code

        Launcher.launch(Map.coDriver.getRawButton(6));

        // .Elevator.run(Map.coDriver.getRawButtonPressed(1),
        // Map.coDriver.getRawButtonPressed(2), Map.coDriver.getRawButtonPressed(3),
        // Map.coDriver.getRawButtonPressed(4), Map.coDriver.getRawAxis(1));
        Elevator.test(Map.coDriver.getRawButtonPressed(1), Map.coDriver.getRawAxis(5),
                Map.coDriver.getRawButtonPressed(2), Map.coDriver.getRawButtonPressed(3));

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
        // Intake.run(Map.driver.getRawButton(6),Map.coDriver.getRawButtonPressed(4),Map.coDriver.getRawButton(6),Map.coDriver.getRawButton(7),false);

        Launcher.run(
                // change for testing
                Map.coDriver.getRawButtonPressed(9),
                Map.coDriver.getRawButton(7),
                Launcher.manualAngleTuner(Map.coDriver.getPOV()), false);
    }

    /**
     * Exit code for teleop mode (overrides IterativeRobotBase.teleopExit)
     */
    @Override
    public void teleopExit() {
    }

    /**
     * Initialization code for disabled mode (overrides
     * IterativeRobotBase.disabledInit)
     */
    @Override
    public void disabledInit() {
    }

    /**
     * Periodic code for disabled mode (overrides
     * IterativeRobotBase.disabledPeriodic)
     */
    @Override
    public void disabledPeriodic() {
        Map.swerve.disabled();
        // Map.swerve.disabledPos();
        Launcher.disable(Map.coDriver.getRawButton(10));
        Intake.disable(Map.coDriver.getRawButton(10));

        Elevator.disable(Map.coDriver.getRawButton(10));
    }

    /**
     * Exit code for disabled mode (overrides IterativeRobotBase.disabledExit)
     */
    @Override
    public void disabledExit() {
    }

    /**
     * Initialization code for test mode (overrides IterativeRobotBase.testInit)
     */
    @Override
    public void testInit() {
    }

    /**
     * Periodic code for test mode (overrides IterativeRobotBase.testPeriodic)
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * Exit code for test mode (overrides IterativeRobotBase.testExit)
     */
    @Override
    public void testExit() {
    }

    /**
     * Robot-wide simulation initialization code (overrides
     * IterativeRobotBase.simulationInit)
     */
    @Override
    public void simulationInit() {
    }

    /**
     * Periodic simulation code (overrides IterativeRobotBase.simulationPeriodic)
     */
    @Override
    public void simulationPeriodic() {
    }

}
