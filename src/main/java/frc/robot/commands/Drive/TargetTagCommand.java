package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class TargetTagCommand extends Command {
  public static boolean target = false;
  public static double freeze = 0;

  public TargetTagCommand() {}

  @Override
  public void initialize() {
    // The additional rotation is because Euclid's shooter sends notes off-centerline due to the
    // grip/slip dichotomy of the wheels.
    freeze = Drive.getSpeakerYaw().plus(new Rotation2d(Units.degreesToRadians(5))).getDegrees();
    Logger.recordOutput("Targeting/TargetTagFreeze", freeze);
  }

  @Override
  public void execute() {

    target = true;
  }

  @Override
  public void end(boolean interrupted) {
    target = false;
    freeze = 0;
  }
}
