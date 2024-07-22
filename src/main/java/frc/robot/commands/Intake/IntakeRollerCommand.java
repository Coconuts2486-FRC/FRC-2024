package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRollers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeRollerCommand extends Command {
  private final Intake intake;
  private final IntakeRollers intakeRollers;
  private final DoubleSupplier mIntake;
  private final DoubleSupplier mOuttake;
  private final BooleanSupplier lightStop;

  public IntakeRollerCommand(IntakeRollers intakeRollers,
      Intake intake, DoubleSupplier mIntake, DoubleSupplier mOuttake, BooleanSupplier lightStop) {
    this.mIntake = mIntake;
    this.intake = intake;
    this.mOuttake = mOuttake;
    this.lightStop = lightStop;
    this.intakeRollers = intakeRollers;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeRollers.setRollers(
        .3, .35, mIntake.getAsDouble(), mOuttake.getAsDouble(), lightStop.getAsBoolean());
    if (lightStop.getAsBoolean()) {}
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
