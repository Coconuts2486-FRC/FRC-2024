package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

/**
 * AutoPaths Control Class *
 */
@SuppressWarnings("removal")
public class Auto {

    public static int a = 0;
    public static double rand = 0;
    public static double holdYaw;
    public static double autoYaw;
    public static double delay;
    public static double startTime;
    public static PIDController twistPid = new PIDController(0.0095, 0, 0.000002);
    public static PIDController holdPid = new PIDController(0.02, 0, 0.0000);

    // Change names later
    public static void init() {
        startTime = 0;
        a = 0;

    }

    /**
     * AUTO: 2-piece; straight motion from spaker
     * 
     * @param red Are we red alliance?
     */
    public static void twoPieceStraightFromSpeaker(boolean red) {

        // Step #0: Initialize everything
        if (a == 0) {
            // Map.swerve.realignToField(true);
                   Swerve.reZeroPosition();
            Map.odometry.init();
            if (red) {
                Swerve.gyro.setYaw(240);
            } else {
                Swerve.gyro.setYaw(120);
            }
            a = 1;
        }

        // Step #1: Shoot the pre-loaded piece
        else if (a == 1) {
           Intake.auto(false, false, true, false, false, false, 0, 0, false, red);
            Launcher.launch(true);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 18100) {
                Timer.delay(.3);
                delay = Timer.getFPGATimestamp();

                a = 2;
            }
        }

        // Step #2: Drive back a bit before looking for the next game piece
        else if (a == 2) {
          if (red){
              Launcher.launch(false);
            Map.swerve.drive(-0.3, 0, 0, false);
            if (Timer.getFPGATimestamp()-delay>2) {
                delay=Timer.getFPGATimestamp();
                Map.swerve.drive(0, 0, 0, false);
                a = 3;
          

          }  }else {
              Launcher.launch(false);
            Map.swerve.drive(0.3, 0, 0, false);
            if (Timer.getFPGATimestamp()-delay>4) {
                delay=Timer.getFPGATimestamp();
                Map.swerve.drive(0, 0, 0, false);
                a = 3;
                
            }
          
          

        }
    }

        // Step #3: Target game piece and intake
        else if (a == 3) {
          
            Launcher.launch(false);
                        Map.swerve.drive(0, -.30, 0, false);
            if (Timer.getFPGATimestamp()-delay>3) {
                Map.swerve.drive(0, 0, 0, false);
                a = 100;
            }
        }

        // Step #4: Line up on the SPEAKER CENTER AprilTag
        else if (a == 4) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, -0,
                    Misc.getSelectedColor()), false);
            RaspberryPi.targetGamePiece(false, false);
            if (Math.abs(RaspberryPi.getSpeakerCenterX(red)) < 4) {
                Map.swerve.drive(0, 0, 0, false);
                a = 5;
            }
        }

        // Step #5: Use the regression to align the pivot for this distance
        else if (a == 5) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))) < .1) {
                System.out.println(
                        Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))));
                a = 6;
            }
        }

        // Step #6: Launch the game piece
        else if (a == 6) {
            System.out.println(
                    Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))));
           Intake.auto(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 18100) {
                a = 7;
            }
        }

        // Step #7: ALL STOP
        else if (a == 100) {
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0., .0, 0, false);
        }
    }

    /**
     * AUTO: 4-piece; Pre-Load + 3 close-in pieces
     * 
     * @param red Are we red alliance?
     */
    public static void fourPieceStraightFromSpeaker(boolean red) {

        // Step #0: Initialize everything
        if (a == 0) {
            Map.swerve.realignToField(true);
            autoYaw = Swerve.gyro.getYaw();
            a = 1;
            Swerve.reZeroPosition();
            Map.odometry.init();
        }

        // Step #1: Shoot the pre-loaded piece
        else if (a == 1) {
           Intake.auto(false, false, true, false, false, false, 0, 0, false, red);
            Launcher.launch(true);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 17800) {
                Map.leftIntake.set(ControlMode.PercentOutput, 1);
                Map.rightIntake.set(ControlMode.PercentOutput, 1);
                Timer.delay(.2);
                a = 2;
            }
        }

        // Step #2: Drive back a bit before looking for the next game piece
        else if (a == 2) {
           Intake.auto(true, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, -0.4, 0, false);
            if (Map.odometry.calculatePosition()[1] >= 4000000) {
                Map.swerve.drive(0, 0, 0, false);
                a = 3;
            }

        }

        // Step #3: Target game piece and intake
        else if (a == 3) {
           Intake.auto(true, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(true, false);
            if (Map.lightStop.get()) {
                Map.swerve.drive(0, 0, 0, false);
                a = 4;
            }
        }

        // Step #4: Line up on the SPEAKER CENTER AprilTag
        else if (a == 4) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, -0,
                    Misc.getSelectedColor()), false);
            RaspberryPi.targetGamePiece(false, false);
            if (Math.abs(RaspberryPi.getSpeakerCenterX(red)) < 4) {
                Map.swerve.drive(0, 0, 0, false);
                a = 5;
            }
        }

        // Step #5: Use the regression to align the pivot for this distance
        else if (a == 5) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, -0.1, true, true);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red)-.1)) < .1) {
                System.out.println(
                        Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))));
                a = 6;
            }
        }

        // Step #6: Launch the game piece
        else if (a == 6) {
            System.out.println(
                    Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))));
           Intake.auto(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, -.1, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 18000) {
                a = 7;
            }
        }

        // Step #7: After the shot, drive at an angle to target the next game piece
        else if (a == 7) {
            // System.out.println(Timer.getFPGATimestamp()-Robot.autoStart);
           Intake.auto(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, -.2, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() < 17800) {
                Map.swerve.drive(-0.2598, .15, 0, false);
                Map.odometry.init();
                a = 8;
            }
        }

        // Step #8: Watch odometry and stop driving when we get far enough
        else if (a == 8) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(-0.2598, .15, 0, false);
            if (Map.odometry.calculatePosition()[1] <= -20500000) {
                Map.swerve.drive(0, .0, 0, false);
                a = 9;
            }
        }

        // Step #9: Target the game peice and intake
        else if (a == 9) {
           Intake.auto(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(true, false);
            if (Map.lightStop.get()) {
                Map.swerve.drive(0, 0, 0, false);
                System.out.println(Timer.getFPGATimestamp() - Robot.autoStart);
                a = 10;
            }
        }

        // Step #10: Rotate toward the SPEAKER
        else if (a == 10) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, .25, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() >= 195) {
                Map.swerve.drive(0, 0, 0, false);
                delay = Timer.getFPGATimestamp();
                a = 11;
            }
        }

        // Step #11: Pause briefly to allow the camera to acquire the AprilTag
        else if (a == 11) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, 0, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Timer.getFPGATimestamp() >= delay + .2) {
                a = 12;
            }
        }

        // Step #12: Line up on the SPEAKER CENTER AprilTag
        else if (a == 12) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, -0,
                    Misc.getSelectedColor()), false);
            RaspberryPi.targetGamePiece(false, false);
            if (Math.abs(RaspberryPi.getSpeakerCenterX(red)) < 4) {
                Map.swerve.drive(0, 0, 0, false);
                a = 13;
            }
        }

        // Step #13: Use the regression to align the pivot for this distance
        else if (a == 13) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))) < .1) {
                System.out.println(
                        Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))));
                a = 14;
            }
        }

        // Step #14: Launch the game piece
        else if (a == 14) {
            System.out.println(
                    Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))));
           Intake.auto(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 18000) {
                a = 15;
            }
        }

        // Step #15: After the shot, drive in ROBOT CENTRIC Y before rotating
        else if (a == 15) {
           Intake.auto(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() < 17800) {
                Map.swerve.drive(.4 * Math.cos(Math.toRadians(15)), 0.4 * Math.sin(Math.toRadians(15)), 0, false);
                Map.odometry.init();
                System.out.println(Timer.getFPGATimestamp() - Robot.autoStart);
                a = 16;
            }
        }

        // Step #16: Watch odometry and stop driving when we get far enough
        // NOTE: The Y velocity is OPPOSITE SIGN than Step #15 (not the case
        // for Steps #7 & #8)
        else if (a == 16) {
           Intake.auto(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(.4 * Math.cos(Math.toRadians(15)), 0.4 * Math.sin(Math.toRadians(15)), 0, false);
            if (Map.odometry.calculatePosition()[1] >= 13300000) {
                Map.swerve.drive(0, 0, 0, false);

                a = 17;
            }
        }

        // Step #17: Rotate so that we can target the next game piece
        else if (a == 17) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, -.3, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() <= 156) {
                Map.swerve.drive(0, 0, 0, false);
                a = 18;
            }
            // Step #18: pick up next note
        } else if (a == 18) {
           Intake.auto(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, Math.signum(RaspberryPi.gamePieceX()) * .079, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Math.abs(RaspberryPi.gamePieceX()) < 4) {
                System.out.println("done " + RaspberryPi.gamePieceX());
                Map.swerve.drive(0, 0, 0, false);

                a = 19;
            }
        } else if (a == 19) {
           Intake.auto(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);

            RaspberryPi.targetGamePiece(true, false);
            if (Map.lightStop.get()) {
                Map.swerve.drive(0, 0, 0, false);

                a = 20;
            }
        } else if (a == 20) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, .3, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() >= 150) {
                Map.swerve.drive(0, 0, 0, false);
                delay = Timer.getFPGATimestamp();
                a = 21;
            }
        }

        // Step #11: Pause briefly to allow the camera to acquire the AprilTag
        else if (a == 21) {
           Intake.auto(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, 0, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Timer.getFPGATimestamp() >= delay + .1) {
                a = 22;
            }
        }

        // Step #12: Line up on the SPEAKER CENTER AprilTag
        else if (a == 22) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, -0,
                    Misc.getSelectedColor()), false);
            RaspberryPi.targetGamePiece(false, false);
            if (Math.abs(RaspberryPi.getSpeakerCenterX(red)) < 4) {
                Map.swerve.drive(0, 0, 0, false);
                a = 23;
            }
        }

        // Step #13: Use the regression to align the pivot for this distance
        else if (a == 23) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))) < .1) {
                System.out.println(
                        Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))));
                a = 24;
            }
        }

        // Step #14: Launch the game piece
        else if (a == 24) {
            System.out.println(
                    Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))));
           Intake.auto(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 18000) {
                a = 25;
            }
        } else if (a == 25) {
           Intake.auto(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() < 17800) {
                System.out.println(Timer.getFPGATimestamp() - Robot.autoStart);
                a = 100;
            }
        }
        // Step #19: ALL STOP
        else if (a == 100) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
        }
    }

    /**
     * AUTO: 3-piece; Pre-Load + 2 center line
     * 
     * @param red Are we red alliance?
     */
    public static void threePieceCenterLine(boolean red) {
        rand = Math.random();
        // set gyro (need a red and blue option) 180 +- 60 degrees
        if (a == 0) {
            Swerve.reZeroPosition();
            Map.odometry.init();
            if (red) {
                Swerve.gyro.setYaw(240);
            } else {
                Swerve.gyro.setYaw(120);
            }
            a = 1;
        } else if (a == 1) {

           Intake.auto(false, false, true, false, false, false, 0, 0, false, red);
            Launcher.launch(true);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 17800) {
                Map.leftIntake.set(ControlMode.PercentOutput, 1);
                Map.rightIntake.set(ControlMode.PercentOutput, 1);
                Timer.delay(.2);

                a = 2;
            }

        } else if (a == 2) {
           
            if(red){
                 System.out.println(Map.odometry.calculatePosition()[1]);
           Intake.auto(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
                  Map.swerve.drive(-.7 * Math.cos(Math.toRadians(60)), -0.7 * Math.sin(Math.toRadians(60)), holdPid.calculate(Swerve.gyro.getYaw(),240), false);
                  if (Map.odometry.calculatePosition()[1] >= 39000000) {

                Map.swerve.drive(0, 0, 0, false);
                delay = Timer.getFPGATimestamp();
                a = 3;
            }
            }else{
                 System.out.println(Map.odometry.calculatePosition()[1]);
           Intake.auto(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
                   Map.swerve.drive(.7 * Math.cos(Math.toRadians(60)), -0.7 * Math.sin(Math.toRadians(60)), holdPid.calculate(Swerve.gyro.getYaw(),240), false);
                   if (Map.odometry.calculatePosition()[1] >= 39000000) {

                Map.swerve.drive(0, 0, 0, false);
                delay = Timer.getFPGATimestamp();
                a = 3;
            }
            }
          
            
        } else if (a == 3) {
           Intake.auto(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, 0, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Timer.getFPGATimestamp() >= delay + .2) {
                a = 4;
            }
        } else if (a == 4) {
       
        if(red)   { Intake.auto(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, false, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, -.3, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() <= 195) {
                System.out.println();
                // Map.odometry.init();
                Map.swerve.drive(0, 0, 0, false);

                a = 5;
            }
            }else{
                Launcher.run(false, false, 0, false, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, .3, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() >= 195) {
                System.out.println();
                // Map.odometry.init();
                Map.swerve.drive(0, 0, 0, false);

                a = 5;
            }

            }

        } else if (a == 5) {
      
            System.out.println(RaspberryPi.gamePieceZ());
           Intake.auto(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(true, false);
            if (Map.lightStop.get()) {
                // Map.swerve.reZeroPosition();
                Map.swerve.drive(.4 * Math.cos(Math.toRadians(64)), 0.4 * Math.sin(Math.toRadians(64)), 0, false);
                // Map.swerve.autoInit();
                holdYaw = Swerve.gyro.getYaw();
                Map.odometry.init();
                a = 6;
            }

        } else if (a == 6) {
            if (red){
           Intake.auto(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(.65 * Math.cos(Math.toRadians(60)), 0.65 * Math.sin(Math.toRadians(60)), holdPid.calculate(Swerve.gyro.getYaw(),holdYaw), false);
            if (Map.odometry.calculatePosition()[1] <= -43000000) {
                System.out.println(Map.odometry.calculatePosition()[1]);

                Map.swerve.drive(0, 0, 0, false);
                System.out.println(Timer.getFPGATimestamp() - Robot.autoStart);
                delay = Timer.getFPGATimestamp();
                a = 7;
            }
            }else {
                 Intake.auto(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(-.65 * Math.cos(Math.toRadians(60)), 0.65 * Math.sin(Math.toRadians(60)), holdPid.calculate(Swerve.gyro.getYaw(),holdYaw), false);
            if (Map.odometry.calculatePosition()[1] <= -43000000) {
                System.out.println(Map.odometry.calculatePosition()[1]);

                Map.swerve.drive(0, 0, 0, false);
                System.out.println(Timer.getFPGATimestamp() - Robot.autoStart);
                delay = Timer.getFPGATimestamp();
                a = 7;
            }
            }
        } else if (a == 7) {

           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, 0, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Timer.getFPGATimestamp() >= delay + .2) {
                a = 8;
            }

        } else if (a == 8) {
            if(red){
           Intake.auto(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, false, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, .3, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() >= 210) {
                System.out.println();
                // Map.odometry.init();
                Map.swerve.drive(0, 0, 0, false);

                a = 100;
            }
            }else{
                  Intake.auto(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, false, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, -.3, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() <= 210) {
                System.out.println();
                // Map.odometry.init();
                Map.swerve.drive(0, 0, 0, false);

                a = 100;
            }
        }
        } else if (a == 100) {
           Intake.auto(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, false, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
        }
    }
}
    // shoot pre-load
    // drive robot-centric to outer wall
    // rotate to straight downfield
    // drive foreward in field centric to some point
    // target note
    // strafe to shoot-range
    // rotate to tag-ish
    // allign to tag
    // regress
    // start shoot
    // stop shoot
    // reverse strafe ~3/4 of the foreward strafe
    // rotate right about 90 degrees
    // target note
    // back-up to avoid stage (robot centric)
    // strafe to launch-zone
    // rotate to about the speaker
    // target april tag
    // regress
    // start shoot
    // stop shoot
    // stop auto

