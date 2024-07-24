package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class PivotCommand extends Command {
  private final Pivot pivot;
  private final DoubleSupplier angle;
  double anglee = 0;

  public PivotCommand(Pivot pivot, DoubleSupplier angle) {
    this.angle = angle;
    this.pivot = pivot;
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
