package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeRollerCommand extends Command {
  private final IntakeRollers intakeRollers;
  private final DoubleSupplier mIntake;
  private final DoubleSupplier mOuttake;
  private final BooleanSupplier lightStop;

  public IntakeRollerCommand(
      IntakeRollers intakeRollers,
      DoubleSupplier mIntake,
      DoubleSupplier mOuttake,
      BooleanSupplier lightStop) {
    this.mIntake = mIntake;
    this.mOuttake = mOuttake;
    this.lightStop = lightStop;
    this.intakeRollers = intakeRollers;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putBoolean("note", lightStop.getAsBoolean());
    if (lightStop.getAsBoolean()) {
      intakeRollers.setRollers(
          mOuttake.getAsDouble() - mIntake.getAsDouble(),
          mOuttake.getAsDouble() - mIntake.getAsDouble());
    } else {
      intakeRollers.setRollers(
          .3 + mOuttake.getAsDouble() - mIntake.getAsDouble(),
          .35 + mOuttake.getAsDouble() - mIntake.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeRollers.stop();
  }
}
