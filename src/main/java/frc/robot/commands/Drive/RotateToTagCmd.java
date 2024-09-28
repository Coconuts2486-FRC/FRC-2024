package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class RotateToTagCmd extends Command {
  private final Drive drive;
  public static double freeze = 0;
  public static double sight = 0;
  private double pivotOffset;
  private static final PIDController rotatePid = new PIDController(.2, 0, 0.0005);
  private static final PIDController noteTargetPid = new PIDController(3, 0, 0.0005);

  public RotateToTagCmd(Drive drive, double pivotOffset) {
    this.drive = drive;
    this.pivotOffset = pivotOffset;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    int redOffset = (DriverStation.getAlliance().get() == Alliance.Red) ? -1 : 1;

    rotatePid.enableContinuousInput(0, 360);
    freeze =
        Drive.getSpeakerYaw()
            .plus(new Rotation2d(Units.degreesToRadians(5 + redOffset + pivotOffset)))
            .getDegrees();
    sight = Drive.getSpeakerYaw().getRotations();
    Logger.recordOutput("Targeting/TargetTagFreeze", freeze);
  }

  @Override
  public void execute() {
    System.out.println(Drive.getSpeakerYaw().getRotations());


    if (Math.abs(sight) > 999) {
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

              if(DriverStation.getAlliance().get() == Alliance.Red){
 drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0,
              0,
              rotatePid.calculate(180),
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    
    
              } else {
 drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0,
              0,
              rotatePid.calculate(0),
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    
    
              }

     
            } else {
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0,
              0,
              rotatePid.calculate(drive.gyroAngles().getDegrees() - freeze),
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
