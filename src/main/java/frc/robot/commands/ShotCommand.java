package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.intake.Intake;

public class ShotCommand extends Command{
    private final Flywheel flywheel;
    private final Intake intake;
    public ShotCommand(Intake intake, Flywheel flywheel) {
        this.flywheel = flywheel;
        this.intake = intake;
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        flywheel.setDutyCycle(1);

        if (){}
    }
  
    @Override
    public void end(boolean interrupted) {}
}
