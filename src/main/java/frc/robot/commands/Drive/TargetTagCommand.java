package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TargetTagCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  public TargetTagCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.ySupplier = ySupplier;
    this.xSupplier = xSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putNumber("Speaker Yaw", drive.getSpeakerYaw().getDegrees());
    SmartDashboard.putNumber("gyro Yaw", drive.gyroAngles().getDegrees());
    Logger.recordOutput("Gyro/Yaw", drive.gyroAngles().getDegrees());
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DriveCommands.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Omega is the difference between the gyro YAW and the desired speaker YAW
    // BUT: As used in the drive command, omega is the ROTATIONAL VELOCITY, not the desired
    //      position offset.  When targeting, we need to override the OMEGA from the drive
    //      command with a PID/Feedforward combo that sets the YAW of the robot to the
    //      YAW of the speaker!
    // See
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/profilepid-subsystems-commands.html
    double omega = drive.gyroAngles().minus(drive.getSpeakerYaw()).getDegrees();

    // Square both the linearMagnitude and omega, preseriving sign
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field relative speeds & send command
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec(),
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
