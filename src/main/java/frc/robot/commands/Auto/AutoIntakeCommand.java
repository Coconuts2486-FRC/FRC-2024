package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.sma.SmaIntakeRollers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoIntakeCommand extends Command {
  // private final IntakeRollers intakeRollers;
  //The normal intake rollers don't work in auto for some reason so use smaIntakeRollers instead
  private final SmaIntakeRollers smaIntakeRollers;
  private final Intake intake;
  private final BooleanSupplier lightStop;
  private final BooleanSupplier intakeStop;
  private final Pivot pivot;
  private final DoubleSupplier angle;
  double anglee = 0;
  //AutoRoto is not longer used for pivot
  // this is for telling RotationOverride when to turn to notes in auto
  public static int AutoRoto;

  public AutoIntakeCommand(
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
    AutoRoto = 0;
  }

  @Override
  public void execute() {
    //This makes it so the intake retracts when the lightstop is triggered and the command the end when the intake limit switch is triggered
    if (lightStop.getAsBoolean()) {
      if (intakeStop.getAsBoolean()) {
        intake.stop();
        smaIntakeRollers.stop();
        smaIntakeRollers.autoShot(0);
        pivot.holdPosition(angle.getAsDouble());
        end(true);
        AutoRoto = 0;
      } else {
        intake.retract();
        smaIntakeRollers.autoShot(0);
        AutoRoto = 0;
      }
    } else {
      intake.setExtendPosition(46.8);//position of pivot
      smaIntakeRollers.autoShot(0.23);//speed of intake rollers
      pivot.holdPosition(angle.getAsDouble());
      AutoRoto = 1;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    smaIntakeRollers.stop();
    AutoRoto = 0;
  }
}
