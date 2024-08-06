package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeRollers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ManualRollerCmd {
  private ManualRollerCmd() {}

  // negitive is up
  public static Command manualRoller(
      IntakeRollers intakeRollers,
      DoubleSupplier inAxis,
      DoubleSupplier outAxis,
      BooleanSupplier lightStop) {
    return Commands.run(
        () -> {
          SmartDashboard.putBoolean("note", lightStop.getAsBoolean());
          intakeRollers.setRollers(
              inAxis.getAsDouble() - outAxis.getAsDouble(),
              inAxis.getAsDouble() - outAxis.getAsDouble());
        },
        intakeRollers);
  }
}
