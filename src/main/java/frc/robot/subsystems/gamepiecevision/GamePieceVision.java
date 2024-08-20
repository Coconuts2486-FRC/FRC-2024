// Copyright (c) 2024 FRC 2486
// http://github.com/Coconuts2486-FRC
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

package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.gamepiecevision.GamePieceVisionIO.GamePieceVisionIOInputs;
import frc.robot.util.GeomUtil;
import frc.robot.util.VirtualSubsystem;
import lombok.experimental.ExtensionMethod;

/** Vision subsystem for GamePiece vision (built upon a virtual subsystem) */
@ExtensionMethod({GeomUtil.class})
public class GamePieceVision extends VirtualSubsystem {

  private final GamePieceVisionIO[] io;
  private final GamePieceVisionIOInputs[] inputs;
  public static Pose2d gamePieceRelativePose;
  public static double gamePieceRelYaw;

  // Class method definition, including inputs
  public GamePieceVision(GamePieceVisionIO... io) {
    this.io = io;
    inputs = new GamePieceVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new GamePieceVisionIOInputs();
    }
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }

    // Loop over cameras
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      var timestamp = inputs[instanceIndex].timestamp;
      var latency = inputs[instanceIndex].latency;

      // Produce a Pose2d for the gamepiece relative to the robot
      gamePieceRelativePose =
          new Pose2d(inputs[instanceIndex].gpX, inputs[instanceIndex].gpY, new Rotation2d());
      gamePieceRelYaw = inputs[instanceIndex].gpAng;
      if (inputs[instanceIndex].gpAng == -999) {
        gamePieceRelativePose = null;
      }
    }
  }
}
