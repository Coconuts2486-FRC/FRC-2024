package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

public class ShotCommand extends Command {
  private final Flywheel flywheel;
  private final Intake intake;

  public ShotCommand(Intake intake, Flywheel flywheel) {
    this.flywheel = flywheel;
    this.intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    flywheel.setDutyCycle(1);

    if (flywheel.getVelocity() > 68.36) {
      intake.intakeRoller(0, 0, 1, 0, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    flywheel.stop();
  }
}
