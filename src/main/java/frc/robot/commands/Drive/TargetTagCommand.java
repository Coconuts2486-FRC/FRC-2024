package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class TargetTagCommand extends Command {
  public static boolean target = false;
  public static double freeze = 0;

  public TargetTagCommand() {}

  @Override
  public void initialize() {
    freeze = Drive.getSpeakerYaw().getDegrees();
  }

  @Override
  public void execute() {

    target = true;
  }

  @Override
  public void end(boolean interrupted) {
    target = false;
    freeze = 0;
  }
}
