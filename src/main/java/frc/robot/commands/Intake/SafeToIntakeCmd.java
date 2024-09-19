package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class SafeToIntakeCmd extends Command {
  private final Intake intake;

  public SafeToIntakeCmd(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    IntakeExtendCommand.safeToIntake = true;
  }

  @Override
  public void execute() {
    IntakeExtendCommand.safeToIntake = true;
  }

  @Override
  public void end(boolean interrupted) {
    IntakeExtendCommand.safeToIntake = true;
  }
}
