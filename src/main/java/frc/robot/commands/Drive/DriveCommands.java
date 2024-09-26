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
import edu.wpi.first.math.util.Units;
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
          gyroYaw = drive.gyroAngles().getDegrees();
          rotatePid.enableContinuousInput(0, 360);

          // Convert joystick commands to "Always Blue Origin" reference frame (i.e., invert for red
          // to match Gyro)
          // Apply deadband
          int isRed = (DriverStation.getAlliance().get() == Alliance.Red) ? 1 : 0;
          double xDrive = xSupplier.getAsDouble() * Math.pow(-1, isRed);
          double yDrive = ySupplier.getAsDouble() * Math.pow(-1, isRed);

          // Apply deadband
          double speakerYawVal;
          double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xDrive, yDrive), DEADBAND);
          Rotation2d linearDirection = new Rotation2d(xDrive, yDrive);
          double omega;
          //  SmartDashboard.putNumber("Speaker Yaw", Drive.getSpeakerYaw().getDegrees());
          omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square both the linearMagnitude and omega, preseriving sign
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();
          if (DriverStation.getAlliance().get() == Alliance.Red) {
            if (Drive.getSpeakerYaw().getDegrees() < 0) {
              speakerYawVal = Drive.getSpeakerYaw().getDegrees() + 360;
            } else {
              speakerYawVal = Drive.getSpeakerYaw().getDegrees();
            }
          } else {
            if (Drive.getSpeakerYaw().getDegrees() < 0) {
              speakerYawVal = Drive.getSpeakerYaw().getDegrees() + 360;
            } else {
              speakerYawVal = Drive.getSpeakerYaw().getDegrees();
            }
          }
          SmartDashboard.putBoolean("Light Stop", lightStop.getAsBoolean());
          SmartDashboard.putBoolean("target", TargetTagCommand.target);
          SmartDashboard.putNumber("Speaker Yaw@", TargetTagCommand.freeze);
          SmartDashboard.putNumber("Gyro", drive.gyroAngles().getDegrees());
          SmartDashboard.putNumber(
              "Calculate target", drive.gyroAngles().getDegrees() - speakerYawVal);
          SmartDashboard.putNumber("real Turn", omega * drive.getMaxAngularSpeedRadPerSec());
          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          if (TargetTagCommand.target) {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    rotatePid.calculate(drive.gyroAngles().getDegrees() - TargetTagCommand.freeze),
                    isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()));
          } else if (TargetNoteCommand.targetNote) {

            if (lightStop.getAsBoolean()) {
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec(),
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));

            } else {
              if (Intake.getPosition() > 35) {
                // NOTE: THIS IS NOW IN ANGLE RATHER THAN POSITION!!!!!!
                if (Math.abs(
                        Drive.getGamePieceYaw()
                            .minus(new Rotation2d(Units.degreesToRadians(DriveCommands.gyroYaw)))
                            .getDegrees())
                    > 5.0) {
                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          0,
                          0,
                          -noteTargetPid.calculate(Drive.getGamePiecePose().getY()),
                          isFlipped
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation()));
                } else {
                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          1.5 * Math.pow(-1, isRed),
                          0,
                          -noteTargetPid.calculate(Drive.getGamePiecePose().getY()),
                          isFlipped
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation()));
                }
              } else {

                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0,
                        0,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
              }
            }
          } else {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec(),
                    isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()));
          }
        },
        drive);
  }
}
