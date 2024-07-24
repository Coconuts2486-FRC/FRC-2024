package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.IntakeRollers;
import java.util.function.BooleanSupplier;

public class AmpCommand extends Command {
  private final IntakeRollers intakeRollers;
  private final Elevator elevator;
  private final BooleanSupplier lightStop;
  private final BooleanSupplier elevatorTop;
  public static double ampPivot;
  private int index = 0;

  public AmpCommand(
      IntakeRollers intakeRollers,
      Elevator elevator,
      BooleanSupplier lightStop,
      BooleanSupplier elevatorTop) {
    this.intakeRollers = intakeRollers;
    this.elevator = elevator;
    this.lightStop = lightStop;
    this.elevatorTop = elevatorTop;
  }

  @Override
  public void initialize() {
    ampPivot = -10;
  }

  @Override
  public void execute() {
    if (elevatorTop.getAsBoolean()) {
      elevator.stop();
      intakeRollers.setRollers(-1, -1);
    } else {
      elevator.runDutyCycle(-.5);
      if (index == 0) {
        intakeRollers.setRollers(.2, .2);
        if (lightStop.getAsBoolean() == false) {
          index = 1;
        }
      } else if (index == 1) {
        intakeRollers.setRollers(.2, .2);
        if (lightStop.getAsBoolean()) {
          intakeRollers.setRollers(.0, .0);
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    ampPivot = 0;
    index = 0;
  }
}
