package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class DisabledCoast extends Command {
  private final Pivot pivot;
  private final Elevator elevator;
  private final Intake intake;

  public DisabledCoast(Pivot pivot, Elevator elevator, Intake intake) {
    this.intake = intake;
    this.elevator = elevator;
    this.pivot = pivot;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initialize() {
    pivot.coast(true);
    elevator.coast(true);
    intake.coast(true);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    pivot.coast(false);
    elevator.coast(false);
    intake.coast(false);
  }
}
