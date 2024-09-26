package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class TargetNoteCommand extends Command {
  // public static boolean targetNote = false;
  public static double noteYaw = 0;
  public static double saveYaw;

  public TargetNoteCommand() {}

  @Override
  public void initialize() {
    saveYaw = DriveCommands.gyroYaw;
  }

  @Override
  public void execute() {

    Logger.recordOutput("Cmd_Status/GP Tracking", true);
    SmartDashboard.putNumber("note ang", Drive.getGamePieceYaw().getDegrees());
  }

  @Override
  public void end(boolean interrupted) {

    Logger.recordOutput("Cmd_Status/GP Tracking", false);
  }
}
