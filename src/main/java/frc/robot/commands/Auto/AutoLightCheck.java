package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
// This is just makes the intake rollers spin slower so the lightstop actually stops the note

public class AutoLightCheck extends Command {
  // private final IntakeRollers intakeRollers;
  private final BooleanSupplier lightStop;
  public static int LightCheck;

  public AutoLightCheck(BooleanSupplier lightStop) {
    // this.intakeRollers = intakeRollers;
    this.lightStop = lightStop;
  }

  @Override
  public void initialize() {
    int LightCheck = 0;
  }

  @Override
  public void execute() {
    if (lightStop.getAsBoolean()) {
      int LightCheck = 1;
    } else {
      int LightCheck = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
