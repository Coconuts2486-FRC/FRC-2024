package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class AutoRegressedPivotCommand extends Command {
  private final Pivot pivot;
  private final DoubleSupplier angler;
  public static double freezeRegress = 0;

  public AutoRegressedPivotCommand(Pivot pivot, DoubleSupplier angler) {
    this.pivot = pivot;
    this.angler = angler;
  }

  @Override
  public void initialize() {
    freezeRegress = pivot.getSpeakerDistance();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Pivot Angle", pivot.pivotAngle());

    double a = -0.0000035721;
    double b = 0.00256816;
    double c = -0.65009;
    double intercept = 80.2242;
    double angle;
    if (freezeRegress < -100) {
      angle = 60;
    } else {
      angle =
          (a * (freezeRegress * freezeRegress * freezeRegress))
              + (b * freezeRegress * freezeRegress)
              + c * freezeRegress
              + intercept
              - .4;
    }
    // System.out.println(freezeRegress);

    //  System.out.println(angle + angler.getAsDouble());
    pivot.holdPosition(angle + angler.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    freezeRegress = 0;
    pivot.stop();
  }
}
