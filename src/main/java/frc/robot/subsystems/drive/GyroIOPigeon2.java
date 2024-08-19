// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.Drive.TargetNoteCommand;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(14);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(100.0);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    if (TargetNoteCommand.targetNote) {
      inputs.yawPosition =
          Rotation2d.fromDegrees((DriverStation.getAlliance().get() == Alliance.Blue) ? 0 : 180);
    } else {
      inputs.yawPosition =
          Rotation2d.fromDegrees(MathUtil.inputModulus(yaw.getValueAsDouble(), 0, 360));
    }

    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }

  /**
   * Zero the pigeon
   *
   * <p>This method should always rezero the pigeon in ALWAYS-BLUE-ORIGIN orientation. Testing,
   * however, shows that it's not doing what I think it should be doing. There is likely
   * interference with something else in the odometry
   */
  @Override
  public void zero() {
    // With the Pigeon facing forward, forward depends on the alliance selected.
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      System.out.println("Alliance Blue: Setting YAW to 0");
      pigeon.setYaw(0.0);
    } else {
      System.out.println("Alliance Red: Setting YAW to 180");
      pigeon.setYaw(180.0);
    }
  }

  @Override
  public void setAngle(double angle) {
    pigeon.setYaw(angle);
  }
}
