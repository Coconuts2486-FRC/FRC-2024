package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;
import frc.robot.Launcher;
import frc.robot.Map;
import frc.robot.Swerve;
import frc.robot.RaspberryPi;

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
            if (Map.leftLauncher.getSelectedSensorVelocity() > 16000) {
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
            if (Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))) < .05) {
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
            if (Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))) < .05) {
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
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
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
            if (Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))) < .05) {
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
                Map.swerve.drive(0, 0.5, 0, true);
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
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, -0.4, 0, true);
            if (Map.odometry.calculatePosition()[1] <= -4500000) {
                Map.swerve.drive(0, 0, 0, false);
                System.out.println(Timer.getFPGATimestamp() - Robot.autoStart);
                a = 17;
            }
        }

        // Step #17: Rotate so that we can target the next game piece
        else if (a == 17) {
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, -.3, false);
            RaspberryPi.targetGamePiece(false, false);
            if (Swerve.gyro.getYaw() <= 105) {
                Map.swerve.drive(0, 0, 0, false);
                delay = Timer.getFPGATimestamp();
                a = 18;
            }
        }

        // Step #18: ALL STOP
        else if (a == 18) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0., .0, 0, false);
        }
    }
}
