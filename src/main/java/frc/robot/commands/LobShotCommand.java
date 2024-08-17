package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.IntakeRollers;

public class LobShotCommand extends Command {
  private final Flywheel flywheel;
  private final IntakeRollers intakeRollers;

  public LobShotCommand(IntakeRollers intakeRollers, Flywheel flywheel) {
    this.flywheel = flywheel;
    this.intakeRollers = intakeRollers;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    flywheel.setDutyCycle(1);

    if (flywheel.getVelocity() > 40.36) {
      intakeRollers.setRollers(1, 1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeRollers.stop();
    flywheel.stop();
  }
}
