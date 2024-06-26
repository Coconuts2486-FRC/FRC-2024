package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command {
  private final Intake intake;
  private final DoubleSupplier angle;

  public IntakeCommand(Intake intake, DoubleSupplier angle) {
    this.angle = angle;
    this.intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivot.holdPosition(angle.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    pivot.stop();
  }
}
