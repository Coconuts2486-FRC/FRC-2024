package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

public class AutoSpinUpCommand extends Command {
  private final Flywheel flywheel;

  public AutoSpinUpCommand(Flywheel flywheel) {
    this.flywheel = flywheel;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    flywheel.setDutyCycle(1);
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }
}
