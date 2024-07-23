package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ClimbCommand extends Command {
  private final Elevator elevator;
  private final BooleanSupplier stop;
  private final DoubleSupplier upAxis;
  private final DoubleSupplier downAxis;

  public ClimbCommand(
      Elevator elevator, BooleanSupplier stop, DoubleSupplier upAxis, DoubleSupplier downAxis) {
    this.elevator = elevator;
    this.stop = stop;
    this.upAxis = upAxis;
    this.downAxis = downAxis;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Manual DeadBand
    double fixUpAxis;
    double fixDownAxis;
    if (Math.abs(upAxis.getAsDouble()) < .1) {
      fixUpAxis = 0;
    } else {
      fixUpAxis = upAxis.getAsDouble();
    }
    if (Math.abs(downAxis.getAsDouble()) < .1) {
      fixDownAxis = 0;
    } else {
      fixDownAxis = downAxis.getAsDouble();
    }

    // actual command
    if (stop.getAsBoolean() == true) {
      elevator.stop();
    } else {
      elevator.runDutyCycle(-.5 - fixUpAxis + fixDownAxis);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }
}
