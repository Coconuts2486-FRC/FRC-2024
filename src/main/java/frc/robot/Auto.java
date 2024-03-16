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
    public static PIDController twistPid = new PIDController(.006, 0.0, 0);

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

        // Step #0: HHH
        if (a == 0) {
            // Intake.run(false, false, false, false, false, false, 0, 0, false, red);
            // Launcher.run(false, false, false, false, 0, red, false, true,0);
            Map.swerve.realignToField(true);
            autoYaw = Swerve.gyro.getYaw();
            a = 1;
            Swerve.reZeroPosition();
            Map.odometry.init();
        }

        // Step #1: HHH
        else if (a == 1) {
            Intake.run(false, false, true, false, false, false, 0, 0, false, red);
            Launcher.launch(true);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 16000) {
                Timer.delay(.3);
                a = 2;
            }

        }

        // Step #2: HHH
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

        // Step #3: HHH
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

        // Step #4: HHH
        else if (a == 4) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, -0,
                    Misc.getSelectedColor()), false);
            RaspberryPi.targetGamePiece(false, false);
            if (red) {
                if (Math.abs(RaspberryPi.getTagX4()) < 4) {
                    Map.swerve.drive(0, 0, 0, false);
                    a = 5;
                }
            } else {
                if (Math.abs(RaspberryPi.getTagX7()) < 4) {
                    Map.swerve.drive(0, 0, 0, false);
                    a = 5;
                }

            }
        }
        // Step #5: HHH
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

        } else if (a == 6) {
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

        } else if (a == 7) {
            // System.out.println(Timer.getFPGATimestamp()-Robot.autoStart);
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(true);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Map.leftLauncher.getSelectedSensorVelocity() < 15400) {
                Map.odometry.init();
                a = 8;
            }
        } else if (a == 8) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
        }
    }

    public static void threePieceStraightFromSpeaker(boolean red) {

        // Step #0: HHH
        if (a == 0) {
            // Intake.run(false, false, false, false, false, false, 0, 0, false, red);
            // Launcher.run(false, false, false, false, 0, red, false, true,0);
            Map.swerve.realignToField(true);
            autoYaw = Swerve.gyro.getYaw();
            a = 1;
            Swerve.reZeroPosition();
            Map.odometry.init();
        }

        // Step #1: HHH
        else if (a == 1) {
            Intake.run(false, false, true, false, false, false, 0, 0, false, red);
            Launcher.launch(true);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 16000) {
                Timer.delay(.3);
                a = 2;
            }

        }

        // Step #2: HHH
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

        // Step #3: HHH
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

        // Step #4: HHH
        else if (a == 4) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, -0,
                    Misc.getSelectedColor()), false);
            RaspberryPi.targetGamePiece(false, false);
            if (red) {
                if (Math.abs(RaspberryPi.getTagX4()) < 4) {
                    Map.swerve.drive(0, 0, 0, false);
                    a = 5;
                }
            } else {
                if (Math.abs(RaspberryPi.getTagX7()) < 4) {
                    Map.swerve.drive(0, 0, 0, false);
                    a = 5;
                }

            }
        }
        // Step #5: HHH
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

        } else if (a == 6) {
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

        } else if (a == 7) {
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
        } else if (a == 8) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(-0.2598, .15, 0, false);
            if (Map.odometry.calculatePosition()[1] <= -20500000) {
                Map.swerve.drive(0, .0, 0, false);
                a = 9;
            }
        } else if (a == 9) {
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(true, false);
            if (Map.lightStop.get()) {
                Map.swerve.drive(0, 0, 0, false);
             System.out.println(Timer.getFPGATimestamp()-Robot.autoStart);
                a = 10;
            }
        } else if(a==10){
              Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
             Map.swerve.drive(0, 0, .2, false);
            RaspberryPi.targetGamePiece(false, false);

             if (Swerve.gyro.getYaw()>=195) {
             Map.swerve.drive(0, 0, 0, false);
             delay = Timer.getFPGATimestamp();
             if(Timer.getFPGATimestamp()>=delay+.3){
                a=11;
             }
                

            }
        } else if(a==11){
                  Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, -0,
                    Misc.getSelectedColor()), false);
             Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
      
            RaspberryPi.targetGamePiece(false, false);
            if (red) {
                if (Math.abs(RaspberryPi.getTagX4()) < 4.7) {
                    Map.swerve.drive(0, 0, 0, false);
                    a = 12;
                }
            } else {
                if (Math.abs(RaspberryPi.getTagX7()) < 4.7) {
                    Map.swerve.drive(0, 0, 0, false);
                    a = 12;
                }

            }

        }else if(a==12){
        
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, true);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0, 0, 0, false);
            if (Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))) < .05) {
                System.out.println(
                        Math.abs(Launcher.pivotEncoder.getAbsolutePosition() - (Launcher.regressionForAngle(red))));
                a = 13;
            }

        }
        else if (a == 13) {
            System.out.println(Timer.getFPGATimestamp()-Robot.autoStart);
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true, false);
            Launcher.launch(false);
            RaspberryPi.targetGamePiece(false, false);
            Map.swerve.drive(0., .0, 0, false);
        }
    }
}
