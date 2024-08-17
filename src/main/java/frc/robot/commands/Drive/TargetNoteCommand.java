package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class TargetNoteCommand extends Command {
  public static boolean targetNote = false;
  public static double noteYaw = 0;
  public static double saveYaw;

  public TargetNoteCommand() {}

  @Override
  public void initialize() {
    saveYaw = DriveCommands.gyroYaw;
  }

  @Override
  public void execute() {
    targetNote = true;
    SmartDashboard.putNumber("note y", Drive.getGamePiecePose().getY());
    SmartDashboard.putNumber("note x", Drive.getGamePiecePose().getX());
  }

  @Override
  public void end(boolean interrupted) {
    targetNote = false;
  }
}
