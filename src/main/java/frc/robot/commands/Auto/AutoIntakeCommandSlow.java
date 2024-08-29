package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.sma.SmaIntakeRollers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoIntakeCommandSlow extends Command {
  // private final IntakeRollers intakeRollers;
  private final SmaIntakeRollers smaIntakeRollers;
  private final Intake intake;
  private final BooleanSupplier lightStop;
  private final BooleanSupplier intakeStop;
  private final Pivot pivot;
  private final DoubleSupplier angle;
  // for manualling changing pivot angle
  double anglee = 0;
  // this is for telling RotationOverride when to turn to notes in auto
  public static int AutoRotos;

  public AutoIntakeCommandSlow(
      IntakeRollers intakeRollers,
      SmaIntakeRollers smaIntakeRollers,
      Intake intake,
      BooleanSupplier lightStop,
      BooleanSupplier intakeStop,
      Pivot pivot,
      DoubleSupplier angle) {
    // this.intakeRollers = intakeRollers;
    this.smaIntakeRollers = smaIntakeRollers;
    this.intake = intake;
    this.lightStop = lightStop;
    this.intakeStop = intakeStop;
    this.angle = angle;
    this.pivot = pivot;
    addRequirements(intakeRollers);
  }

  @Override
  public void initialize() {
    pivot.holdPosition(angle.getAsDouble());
    System.out.println("Build Issue");
    AutoRotos = 0;
  }

  @Override
  public void execute() {
    if (lightStop.getAsBoolean()) {
      if (intakeStop.getAsBoolean()) {
        intake.stop();
        smaIntakeRollers.stop();
        smaIntakeRollers.autoShot(0);
        pivot.holdPosition(angle.getAsDouble());
        end(true);
        AutoRotos = 0;
      } else {
        intake.retract();
        smaIntakeRollers.autoShot(0);
        AutoRotos = 0;
      }
    } else {
      intake.setExtendPosition(46.8);
      smaIntakeRollers.autoShot(0.175);
      pivot.holdPosition(angle.getAsDouble());
      AutoRotos = 1;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    smaIntakeRollers.stop();
    AutoRotos = 0;
  }
}
