package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotChangerTrueCommand extends Command {
  public static double angley;
  public static int changer;

  public PivotChangerTrueCommand(int changer) {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (changer == 1) {
      angley = angley + .1;
      System.out.println(angley);
    } else if (changer == 2) {
      angley = angley - .1;
      System.out.println(angley);
    } else if (changer == 3) {
      angley = 0;
      System.out.println(angley);
    } else {
      angley = angley;
      System.out.println(angley);
    }
  }

  @Override
  public void end(boolean interrupted) {}
}
