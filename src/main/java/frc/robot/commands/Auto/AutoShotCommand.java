package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.sma.SmaIntakeRollers;

public class AutoShotCommand extends Command {
  private final Flywheel flywheel;
  private final SmaIntakeRollers smaIntakeRollers;
  // Shot Command exepct using smaIntakeRollers so it works for auto

  public AutoShotCommand(
      IntakeRollers intakeRollers, Flywheel flywheel, SmaIntakeRollers smaIntakeRollers) {
    this.flywheel = flywheel;
    this.smaIntakeRollers = smaIntakeRollers;
    addRequirements(intakeRollers);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    flywheel.setDutyCycle(1);

    if (flywheel.getVelocity() > 68.36) {
      smaIntakeRollers.autoShot(1.2);
    }
  }

  @Override
  public void end(boolean interrupted) {
    smaIntakeRollers.stop();
    flywheel.stop();
  }
}
