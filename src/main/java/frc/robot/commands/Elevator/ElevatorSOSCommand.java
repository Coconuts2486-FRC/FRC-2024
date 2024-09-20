package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorSOSCommand extends Command {
  private final Elevator elevator;

  public ElevatorSOSCommand(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.stop();
  }

  @Override
  public void execute() {
    elevator.stop();
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }
}
