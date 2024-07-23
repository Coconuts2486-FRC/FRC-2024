package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotChangerUpCommand extends Command {
  public static double angler;

  public PivotChangerUpCommand() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    angler = angler + .1;
    System.out.println(angler);
  }

  @Override
  public void end(boolean interrupted) {
    angler = angler;
  }
}
