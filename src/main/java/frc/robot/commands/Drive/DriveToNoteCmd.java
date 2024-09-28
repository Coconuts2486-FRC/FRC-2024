package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class DriveToNoteCmd extends Command {
  private final Drive drive;
  public static boolean targetNote = false;
  private final BooleanSupplier lightStop;
  // private static final PIDController rotatePid = new PIDController(.2, 0, 0.0005);
  private static final PIDController noteTargetPid = new PIDController(3, 0, 0.0005);

  public DriveToNoteCmd(Drive drive, BooleanSupplier lightStop) {
    this.drive = drive;
    this.lightStop = lightStop;
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putBoolean("Light Stop", lightStop.getAsBoolean());
    targetNote = true;
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    // int isRed = (DriverStation.getAlliance().get() == Alliance.Red) ? 1 : 0;

    if (lightStop.getAsBoolean()) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0,
              0,
              0,
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    } else {
      if (Intake.getPosition() > 35) {
        // NOTE: THIS IS NOW IN ANGLE RATHER THAN POSITION!!!!!!
        if (Math.abs(
                Drive.getGamePieceYaw()
                    .minus(new Rotation2d(Units.degreesToRadians(DriveCommands.gyroYaw)))
                    .getDegrees())
            > 6.0) {
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
                  2,
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
  }

  @Override
  public void end(boolean interrupted) {
    targetNote = false;
  }
}
