// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  public static final double DEADBAND = 0.1;
  private static final PIDController rotatePid = new PIDController(.2, 0, 0.0005);
  private static final PIDController noteTargetPid = new PIDController(3, 0, 0.0005);
  public static double gyroYaw;
  private static double speakerYawVal;
  private static Rotation2d robotAngle;
  private static boolean isFlipped;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier lightStop) {
    return Commands.run(
        () -> {
          // Get the orientation of the bot based on alliance
          isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          robotAngle =
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();

          // Set the PID to allow continuous input
          rotatePid.enableContinuousInput(0, 360);

          // Get various YAW values needed
          gyroYaw = drive.gyroAngles().getDegrees();
          // This wraps the YAW between 0º and 360º
          speakerYawVal = MathUtil.inputModulus(Drive.getSpeakerYaw().getDegrees(), 0, 360);

          // Convert joystick commands to "Always Blue Origin" reference frame (i.e., invert for red
          // to match Gyro)
          int isRed = (DriverStation.getAlliance().get() == Alliance.Red) ? 1 : 0;
          double xDrive = xSupplier.getAsDouble() * Math.pow(-1, isRed);
          double yDrive = ySupplier.getAsDouble() * Math.pow(-1, isRed);

          // Apply deadbands
          double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xDrive, yDrive), DEADBAND);
          Rotation2d linearDirection = new Rotation2d(xDrive, yDrive);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square both the linearMagnitude and omega, preseriving sign
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Put various values out to the SmartDashboard
          //  SmartDashboard.putNumber("Speaker Yaw", speakerYawVal);
          SmartDashboard.putBoolean("Light Stop", lightStop.getAsBoolean());
          SmartDashboard.putBoolean("target", TargetTagCommand.target);
          SmartDashboard.putNumber("Speaker Yaw@", TargetTagCommand.freeze);
          SmartDashboard.putNumber("Gyro", gyroYaw);
          SmartDashboard.putNumber("Calculate target", gyroYaw - speakerYawVal);
          SmartDashboard.putNumber("real Turn", omega * drive.getMaxAngularSpeedRadPerSec());

          //// Convert to field relative speeds & send command ////

          if (TargetTagCommand.target) {
            // Driving Case #1: Targeting the AprilTag for the Speaker; rotate to speaker YAW
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    rotatePid.calculate(drive.gyroAngles().getDegrees() - TargetTagCommand.freeze),
                    robotAngle));

          } else if (DriveToNoteCmd.targetNote) {
            // Driving Case #2: Driving to the Note; rotate to note YAW and drive toward it
            if (lightStop.getAsBoolean()) {
              // If the Light Stop is triggered, "Fly Casual" -- this is the same as Driving Case #3
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec(),
                      robotAngle));

            } else {
              // If the Light Stop is NOT triggered, we need to go fetch the NOTE!
              if (Intake.getPosition() > 35) {
                // Intake Case #1: Intake is extended
                if (Math.abs(
                        Drive.getGamePieceYaw()
                            .minus(Rotation2d.fromDegrees(DriveCommands.gyroYaw))
                            .getDegrees())
                    > 7.0) {
                  // NOTE Position Case #1: NOTE is >7º from ROBOT FORWARD; rotate toward the note
                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          0,
                          0,
                          // We need a different way of doing this...
                          -noteTargetPid.calculate(Drive.getGamePiecePose().getY()),
                          robotAngle));
                } else {
                  // NOTE Position Case #2: NOTE is <7º from ROBOT FORWARD; rotate and drive forward
                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          1.5 * Math.pow(-1, isRed),
                          0,
                          // We need a different way of doing this...
                          -noteTargetPid.calculate(Drive.getGamePiecePose().getY()),
                          robotAngle));
                }
              } else {
                // Intake Case #2: Intake is NOT extended; just sit there (waiting for the intake to
                // deploy)
                drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, robotAngle));
              }
            }

          } else {
            // Driving Case #3: "Fly Casual"
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec(),
                    robotAngle));
          }
        },
        drive);
  }
}
