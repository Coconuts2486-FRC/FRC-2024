package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotChangerDownCommand extends Command {

  public PivotChangerDownCommand() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    PivotChangerUpCommand.angler = PivotChangerUpCommand.angler - .1;
    System.out.println(PivotChangerUpCommand.angler);
  }

  @Override
  public void end(boolean interrupted) {}
}
