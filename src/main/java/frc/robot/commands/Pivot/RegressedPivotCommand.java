package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class RegressedPivotCommand extends Command {
  private final Pivot pivot;

  public RegressedPivotCommand(Pivot pivot) {
    this.pivot = pivot;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putNumber("Pivot Angle", pivot.pivotAngle());

    double a = -0.0000035721;
    double b = 0.00256816;
    double c = -0.65009;
    double intercept = 80.2242;
    double angle;
    if (pivot.getSpeakerDistance() < -100) {
      angle = 60;
    } else {
      angle =
          (a
                  * (pivot.getSpeakerDistance()
                      * pivot.getSpeakerDistance()
                      * pivot.getSpeakerDistance()))
              + (b * pivot.getSpeakerDistance() * pivot.getSpeakerDistance())
              + c * pivot.getSpeakerDistance()
              + intercept;
    }
    System.out.println(pivot.getSpeakerDistance());

    System.out.println(angle);
    pivot.holdPosition(angle);
  }

  @Override
  public void end(boolean interrupted) {
    pivot.stop();
  }
}
