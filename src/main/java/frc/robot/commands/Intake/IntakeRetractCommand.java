package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class IntakeRetractCommand extends Command {
  private final Intake intake;
  private final BooleanSupplier intakeLimit;

  public IntakeRetractCommand(Intake intake, BooleanSupplier intakeLimit) {
    this.intake = intake;
    this.intakeLimit = intakeLimit;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.retract(intakeLimit.getAsBoolean());
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    intake.setPose(0);
  }
}
