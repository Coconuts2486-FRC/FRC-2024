package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class IntakeExtendCommand extends Command {
  private final Intake intake;
  private final BooleanSupplier lightStop;
  private final BooleanSupplier intakeStop;
  private final BooleanSupplier pivotStop;
  public static boolean safeToIntake = false;

  public IntakeExtendCommand(
      Intake intake,
      BooleanSupplier lightStop,
      BooleanSupplier intakeStop,
      BooleanSupplier pivotStop) {
    this.intake = intake;
    this.lightStop = lightStop;
    this.intakeStop = intakeStop;
    this.pivotStop = pivotStop;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (intakeStop.getAsBoolean()) {
      safeToIntake = true;
    }
    // This is if the pivot is too steep and will hit the swerve module, or the lightstop is
    // triggered, retract
    if (pivotStop.getAsBoolean() || lightStop.getAsBoolean() || safeToIntake == false) {
      if (intakeStop.getAsBoolean()) {
        intake.stop();
      } else {
        intake.retract();
      }
    } else {
      // intake.setExtendPosition(5);
      intake.setExtendPosition(48.8);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
