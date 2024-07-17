package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class AutoIntakeCommand extends Command {
  private final Intake intake;
  private final BooleanSupplier lightStop;
  private final BooleanSupplier intakeStop;

  public AutoIntakeCommand(Intake intake, BooleanSupplier lightStop, BooleanSupplier intakeStop) {
    this.intake = intake;
    this.lightStop = lightStop;
    this.intakeStop = intakeStop;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (lightStop.getAsBoolean()) {
      intake.retract(intakeStop.getAsBoolean());
    } else {
      intake.setExtendPosition(48.8);
      intake.intakeRoller(0.5, 0.5, 0, 0, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
