package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOInputsAutoLogged;

public class Pivot extends SubsystemBase {
    private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
   private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

   public Pivot(PivotIO io) {

   }

}
