package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class IntakeExtendCommand extends Command {
  private final Intake intake;
  private final BooleanSupplier lightStop;
  private final BooleanSupplier intakeStop;

  public IntakeExtendCommand(Intake intake, BooleanSupplier lightStop, BooleanSupplier intakeStop) {
    this.intake = intake;
    this.lightStop = lightStop;
    this.intakeStop = intakeStop;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (lightStop.getAsBoolean()) {
      if (intakeStop.getAsBoolean()) {
        intake.stop();
      } else {
        intake.retract();
      }
    } else {
      intake.setExtendPosition(48.8);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
