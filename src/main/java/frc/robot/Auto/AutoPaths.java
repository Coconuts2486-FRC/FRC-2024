package frc.robot.Auto;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Intake;
import frc.robot.Launcher;
import frc.robot.Map;
import frc.robot.Swerve;
import frc.robot.Vision.RaspberryPi;

/**
 * AutoPaths Control Class *
 */
public class AutoPaths {

    public static int a = 0;
    public static double autoYaw;
    public static double startTime;
    public static PIDController twistPid = new PIDController(.006, 0.0, 0);

    // Change names later
    public static void autoInit() {
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
            Intake.run(false, false, false, false, false, false, 0, 0, false, red);
            // Launcher.run(false, false, false, false, 0, red, false, true,0);
            Map.swerve.realignToField(true);
            autoYaw = Swerve.gyro.getYaw();
            a = a + 1;
        }

        // Step #1: HHH
        else if (a == 1) {
            Launcher.launch(true);
            // Launcher.run(false, false, 0, false);
            // Launcher.run(false, false, false, false, 0, red, false, true,0);
            Intake.run(false, false, true, false, false, false, 0, 0, true, red);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 19560) {
                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                Map.intakeRight.set(ControlMode.PercentOutput, 1);
                Timer.delay(.5);
                a = a + 1;
            }
        }

        // Step #2: HHH
        else if (a == 2) {
            Launcher.run(false, false, 0, true);
            // Launcher.run(false, false, false, false, 0, red, true, false,0);
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            RaspberryPi.targetGamePiece(true, false);
            Launcher.launchAuto(false);
            if (Map.lightStop.get()) {
                a = a + 1;
            }
        } 
        
        // Step #3: HHH
        else if (a == 3) {
            Launcher.run(false, false, 0, true);
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            // Launcher.run(false, false, false, true, 0, red, false, false,0);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, 0, red));
            Launcher.launchAuto(true);
            if (Math.abs(RaspberryPi.getTagX4()) < 7 || Math.abs(RaspberryPi.getTagX7()) < 7) {
                a = a + 1;
            }
        } 
        
        // Step #4: HHH
        else if (a == 4) {
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            // Launcher.run(false, false, false, true, 0, red, false, false,0);
            Launcher.run(false, false, 0, true);
            Launcher.launchAuto(true);
            Map.swerve.drive(0, 0, 0);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 21560) {
                startTime = Timer.getFPGATimestamp();
            }
            if (Map.leftLauncher.getSelectedSensorVelocity() > 20560) {
                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                Map.intakeRight.set(ControlMode.PercentOutput, 1);
                Timer.delay(.4);
                a = a + 1;
            }
        }

        // Step #5: HHH
        else if (a == 5) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.run(false, false, 0, true);
            // Launcher.run(false, false, false, false, 0, red, true, false,0);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, 0);
        }
    }

    /**
     * AUTO: 3-piece; Amp-Side, stay in the zone
     * 
     * @param red Are we red alliance?
     */
    public static void threePieceAmpSideStayInZone(boolean red) {

        if (a == 0) {
            Intake.run(false, false, false, false, false, false, 0, 0, false, red);
            Map.swerve.realignToField(true);
            autoYaw = Swerve.gyro.getYaw();
            if (red) {
                // Red-Side Specific-Code
                Swerve.gyro2.setYaw(autoYaw + 60);
            } else {
                // Blue-Side Specific-Code
                Swerve.gyro2.setYaw(autoYaw - 60);
            }
            a = a + 1;
        }

        // Step #1: HHH
        else if (a == 1) {
            Launcher.launch(true);
            Intake.run(false, false, true, false, false, false, 0, 0, true, red);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 19560) {
                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                Map.intakeRight.set(ControlMode.PercentOutput, -1);
                Timer.delay(.5);
                a = a + 1;
            }
        } 
        
        // Step #2: HHH
        else if (a == 2) {
            Intake.run(false, false, true, false, false, false, 0, 0, true, red);
            Map.swerve.drive(0, -.5, 0);
            if (Math.abs(Map.frontLeft.driveMotor.getSelectedSensorVelocity()) > 100) {
                Timer.delay(.23);
                Map.swerve.drive(0, 0, 0);
                Swerve.reZeroPosition();
                Timer.delay(.23);
                a = a + 1;
            }
        } 
        
        // Step #3: HHH
        else if (a == 3) {
            Launcher.launch(false);
            Intake.run(false, false, true, false, false, false, 0, 0, true, red);
            if (red) {
                // Red-Side Specific-Code
                Map.swerve.drive(0, 0, twistPid.calculate(Swerve.gyro.getYaw(), 240));
                if (Math.abs((240) - Swerve.gyro.getYaw()) < 5) {
                    Timer.delay(.2);
                    Swerve.gyro.setYaw(Swerve.gyro.getYaw());
                    a = 4;
                }
            } else {
                // Blue-Side Specific-Code
                Map.swerve.drive(0, 0, twistPid.calculate(Swerve.gyro.getYaw(), 120));
                if (Math.abs((120) - Swerve.gyro.getYaw()) < 5) {
                    Timer.delay(.2);
                    Swerve.gyro.setYaw(Swerve.gyro.getYaw());
                    a = 4;
                }
            }
        }

        // Step #4: HHH
        else if (a == 4) {
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            RaspberryPi.targetGamePiece(true, false);
            Launcher.launchAuto(false);
            if (Map.lightStop.get()) {
                a = 5;
            }
        } 
        
        // Step #5: HHH
        else if (a == 5) {
            Intake.run(true, false, false, false, false, true, 0, 0, false, red);
            if (red) {
                // Red-Side Specific-Code
                Map.swerve.drive(0, 0, -.3);
                if (Math.abs(Map.frontLeft.driveMotor.getSelectedSensorVelocity()) > 100) {
                    Timer.delay(.2);
                    a = 6;
                }
            } else {
                // Blue-Side Specific-Code
                Map.swerve.drive(0, 0, .3);
                if (Math.abs(Map.frontLeft.driveMotor.getSelectedSensorVelocity()) > 100) {
                    Timer.delay(.2);
                    a = 6;
                }
            }

        } 
        
        // Step #6: HHH
        else if (a == 6) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Map.swerve.drive(0, 0, RaspberryPi.targetAprilTag(true, 0, red));
            Launcher.launchAuto(true);
            if (Math.abs(RaspberryPi.getTagX4()) < 7 || Math.abs(RaspberryPi.getTagX7()) < 7) {
                a = a + 1;
            }
        } 
        
        // Step #7: HHH
        else if (a == 7) {
            Intake.run(false, false, true, false, false, true, 0, 0, false, red);
            Launcher.launchAuto(true);
            Map.swerve.drive(0, 0, 0);
            if (Map.leftLauncher.getSelectedSensorVelocity() > 21560) {
                startTime = Timer.getFPGATimestamp();
            }
            if (Map.leftLauncher.getSelectedSensorVelocity() > 20560) {
                Map.intakeLeft.set(ControlMode.PercentOutput, 1);
                Map.intakeRight.set(ControlMode.PercentOutput, -1);
                Timer.delay(.4);
                a = 8;
            }
        } 
        
        // Step #7: HHH
        else if (a == 7) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Map.swerve.drive(0, 0, twistPid.calculate(Map.swerve.gyro2.getYaw(), 90));
            Timer.delay(1);
            a = a++;
        }

        // Step #8: HHH
        else if (a == 8) {
            Intake.run(false, false, false, false, false, true, 0, 0, false, red);
            Launcher.launch(false);
            Map.swerve.drive(0, 0, 0);
        }
    }

    /**
     * AUTO: Back Up
     * 
     * @param red Are we red alliance?
     */
    public static void backUp(boolean red) {

        Map.swerve.drive(0, -0.29, 0);

    }
}
