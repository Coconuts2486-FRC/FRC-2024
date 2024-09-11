package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class AutoRegressedPivotCommand extends Command {
  private final Pivot pivot;
  private final DoubleSupplier angler;
  private final DoubleSupplier ogAngle;
  public static double freezeRegress = 0;
  // This variable is used to tell RotationOverride when to turn to speaker in auto
  public static int AutoShoto;

  public AutoRegressedPivotCommand(Pivot pivot, DoubleSupplier angler, DoubleSupplier ogAngle) {
    this.pivot = pivot;
    this.angler = angler;
    this.ogAngle = ogAngle;
  }

  @Override
  public void initialize() {
    freezeRegress = pivot.getSpeakerDistance();
    AutoShoto = 0;
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
      angle = 0 + ogAngle.getAsDouble();
    } else {
      angle =
          (a * (freezeRegress * freezeRegress * freezeRegress))
              + (b * freezeRegress * freezeRegress)
              + c * freezeRegress
              + intercept
              - 0.4;
    }
    // System.out.println(freezeRegress);

    //  System.out.println(angle + angler.getAsDouble());
    pivot.holdPosition(angle + angler.getAsDouble());
    AutoShoto = 1;
  }

  @Override
  public void end(boolean interrupted) {
    freezeRegress = 0;
    pivot.stop();
    AutoShoto = 0;
  }
}
