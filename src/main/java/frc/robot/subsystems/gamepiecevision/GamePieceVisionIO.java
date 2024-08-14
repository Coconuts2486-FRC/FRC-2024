// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
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

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GamePieceVisionIO {
  class GamePieceVisionIOInputs implements LoggableInputs {
    public String camname = "";
    public double latency = 0.0;
    public double timestamp = 0.0;
    public double gpX;
    public double gpY;
    public double gpZ;
    public double gpAng;
    public double gpDist;

    public long fps = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Latency", latency);
      table.put("Timestamp", timestamp);
      table.put("GamePieceX", gpX);
      table.put("GamePieceY", gpY);
      table.put("GamePieceZ", gpZ);
      table.put("GamePieceAng", gpAng);
      table.put("GamePieceDist", gpDist);
      table.put("Fps", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      latency = table.get("Latency", 0.0);
      timestamp = table.get("Timestamp", 0.0);
      gpX = table.get("GamePieceX", Float.NaN);
      gpY = table.get("GamePieceY", Float.NaN);
      gpZ = table.get("GamePieceZ", Float.NaN);
      gpAng = table.get("GamePieceAng", Float.NaN);
      gpDist = table.get("GamePieceDist", Float.NaN);
      fps = table.get("Fps", 0);
    }
  }

  default void updateInputs(GamePieceVisionIOInputs inputs) {}
}
