package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class TargetNoteCommand extends Command {
  public static boolean target = false;
  public static double noteYaw = 0;

  public TargetNoteCommand() {}

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber(getName(), Drive.getGamePiecePose().getY());
  }

  @Override
  public void end(boolean interrupted) {
  }
}