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
    public static double autoYaw;
    public static double delay;
    public static double startTime;
    public static PIDController twistPid = new PIDController(0.0095, 0, 0.000002);

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
            Map.swerve.realignToField(true);
            autoYaw = Swerve.gyro.getYaw();
            a = 1;
            Swerve.reZeroPosition();
            Map.odometry.init();
        }

        // Step #1: Shoot the pre-loaded piece
        else if (a == 1) {
            Intake.run(false, false, true, false, false, false, 0, 0, false, red);
            Launcher.launch(true);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 16050) {
                Timer.delay(.3);
                a = 2;
            }
        }

        // Step #2: Drive back a bit before looking for the next game piece
        else if (a == 2) {
            Intake.run(false, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, -0.3, 0, false);
            if (Map.odometry.calculatePosition()[1] >= 4000000) {
                Map.swerve.drive(0, 0, 0, false);
                a = 3;
            }

        }

        // Step #3: Target game piece and intake
        else if (a == 3) {
            Intake.run(true, false, false, false, false, false, 0, 0, false, red);
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
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 16000) {
                a = 7;
            }
        }

        // Step #7: ALL STOP
        else if (a == 7) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
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
            Intake.run(false, false, true, false, false, false, 0, 0, false, red);
            Launcher.launch(true);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 16000) {
                   Map.leftIntake.set(ControlMode.PercentOutput, 1);
            Map.rightIntake.set(ControlMode.PercentOutput, 1);
                Timer.delay(.3);
                a = 2;
            }
        }

        // Step #2: Drive back a bit before looking for the next game piece
        else if (a == 2) {
            Intake.run(true, false, false, false, false, false, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, -0.3, 0, false);
            if (Map.odometry.calculatePosition()[1] >= 4000000) {
                Map.swerve.drive(0, 0, 0, false);
                a = 3;
            }

        }

        // Step #3: Target game piece and intake
        else if (a == 3) {
            Intake.run(true, false, false, false, false, false, 0, 0, false, red);
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
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 16000) {
                a = 7;
            }
        }

        // Step #7: After the shot, drive at an angle to target the next game piece
        else if (a == 7) {
            // System.out.println(Timer.getFPGATimestamp()-Robot.autoStart);
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() < 15400) {
                Map.swerve.drive(-0.2598, .15, 0, false);
                Map.odometry.init();
                a = 8;
            }
        }

        // Step #8: Watch odometry and stop driving when we get far enough
        else if (a == 8) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, .2, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() >= 195) {
                Map.swerve.drive(0, 0, 0, false);
                delay = Timer.getFPGATimestamp();
                a = 11;
            }
        }

        // Step #11: Pause briefly to allow the camera to acquire the AprilTag
        else if (a == 11) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, 0, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Timer.getFPGATimestamp() >= delay + .3) {
                a = 12;
            }
        }

        // Step #12: Line up on the SPEAKER CENTER AprilTag
        else if (a == 12) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 16000) {
                a = 15;
            }
        }

        // Step #15: After the shot, drive in ROBOT CENTRIC Y before rotating
        else if (a == 15) {
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() < 15400) {
                Map.swerve.drive(.4*Math.cos(Math.toRadians(15)), 0.4*Math.sin(Math.toRadians(15)), 0, false);
                Map.odometry.init();
                System.out.println(Timer.getFPGATimestamp() - Robot.autoStart);
                a = 16;
            }
        }

        // Step #16: Watch odometry and stop driving when we get far enough
        // NOTE: The Y velocity is OPPOSITE SIGN than Step #15 (not the case
        // for Steps #7 & #8)
        else if (a == 16) {
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(.4*Math.cos(Math.toRadians(15)), 0.4*Math.sin(Math.toRadians(15)), 0, false);
            if (Map.odometry.calculatePosition()[1] >= 13300000) {
                Map.swerve.drive(0, 0, 0, false);
              
                a = 17;
            }
        }

        // Step #17: Rotate so that we can target the next game piece
        else if (a == 17) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, -.3, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() <= 167) {
                Map.swerve.drive(0, 0, 0, false);
                a = 18;
            }
            // Step #18: pick up next note
         } else if (a == 18){
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
              Map.swerve.drive(0, 0, Math.signum(RaspberryPi.gamePieceX())*.083, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Math.abs(RaspberryPi.gamePieceX())<4) {
                System.out.println("done "+ RaspberryPi.gamePieceX());
                Map.swerve.drive(0, 0, 0, false);

                a = 19;
            }
         }
         else if (a == 19) {
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(true, false);
            if (Map.lightStop.get()) {
                Map.swerve.drive(0, 0, 0, false);

                a = 20;
            }
        } else if (a == 20) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, .3, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() >= 153) {
                Map.swerve.drive(0, 0, 0, false);
                delay = Timer.getFPGATimestamp();
                a = 21;
            }
        }

        // Step #11: Pause briefly to allow the camera to acquire the AprilTag
        else if (a == 21) {
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, 0, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Timer.getFPGATimestamp() >= delay + .2) {
                a = 22;
            }
        }

        // Step #12: Line up on the SPEAKER CENTER AprilTag
        else if (a == 22) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 16000) {
                a = 25;
            }
        }
         else if (a == 25) {
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() < 15400) {
                System.out.println(Timer.getFPGATimestamp() - Robot.autoStart);
                a = 100;
            }
        }
        // Step #19: ALL STOP
        else if (a == 100) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
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
        //set gyro (need a red and blue option) 180 +- 60 degrees
        if (a==0){
            Swerve.reZeroPosition();
            Map.odometry.init();
            if (red){
                Swerve.gyro.setYaw(240);
            }else{
                Swerve.gyro.setYaw(120);
            }
              a = 1;
                }
        //shoot pre-load
        //drive robot-centric to outer wall
        //rotate to straight downfield
        //drive foreward in field centric to some point
        // target note
        // strafe to shoot-range
        // rotate to tag-ish
        // allign to tag
        //regress
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
    }
}
