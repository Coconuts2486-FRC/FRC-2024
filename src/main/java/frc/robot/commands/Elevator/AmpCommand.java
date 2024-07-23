package frc.robot.commands.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.pivot.Pivot;

public class AmpCommand extends Command {
  private final IntakeRollers intakeRollers;
  private final Elevator elevator;
  private final DoubleSupplier lightStop;

  public AmpCommand(IntakeRollers intakeRollers,Elevator elevator, DoubleSupplier lightStop) {
    this.intakeRollers = intakeRollers;
    this.elevator = elevator;
    this.lightStop = lightStop;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}
}
