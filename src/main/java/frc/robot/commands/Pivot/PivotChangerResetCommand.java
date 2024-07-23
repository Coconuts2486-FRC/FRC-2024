package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotChangerResetCommand extends Command {

  public PivotChangerResetCommand() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    PivotChangerUpCommand.angler = 0;
    System.out.println(PivotChangerUpCommand.angler);
  }

  @Override
  public void end(boolean interrupted) {
    PivotChangerUpCommand.angler = PivotChangerUpCommand.angler;
  }
}
