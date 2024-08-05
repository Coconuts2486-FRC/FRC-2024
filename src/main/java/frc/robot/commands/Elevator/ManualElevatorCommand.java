package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ManualElevatorCommand extends Command {
  private final Elevator elevator;
  private final BooleanSupplier topStop;
  private final BooleanSupplier bottomStop;
  private final DoubleSupplier upAxis;
  private final DoubleSupplier downAxis;

  public ManualElevatorCommand(
      Elevator elevator,
      BooleanSupplier topStop,
      BooleanSupplier bottomStop,
      DoubleSupplier upAxis,
      DoubleSupplier downAxis) {
    this.elevator = elevator;
    this.topStop = topStop;
    this.upAxis = upAxis;
    this.downAxis = downAxis;
    this.bottomStop = bottomStop;
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
    if (topStop.getAsBoolean() == true) {
      elevator.runDutyCycle(fixDownAxis);
    } else if (bottomStop.getAsBoolean() == true) {
      elevator.setPose(0);
      elevator.runDutyCycle(-fixUpAxis);
    } else {
      elevator.runDutyCycle(-fixUpAxis + fixDownAxis);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }
}
