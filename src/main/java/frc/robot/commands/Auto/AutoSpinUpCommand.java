package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

public class AutoSpinUpCommand extends Command {
  private final Flywheel flywheel;
  //this is for telling RotationOverride when to turn to speaker in auto
  public static int AutoShoto2;

  public AutoSpinUpCommand(Flywheel flywheel) {
    this.flywheel = flywheel;
  }

  @Override
  public void initialize() {
    AutoShoto2 = 0;
  }

  @Override
  public void execute() {
    flywheel.setDutyCycle(1);
    AutoShoto2 = 1;
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    AutoShoto2 = 0;
  }
}
