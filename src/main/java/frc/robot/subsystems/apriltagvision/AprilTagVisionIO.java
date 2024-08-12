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

package frc.robot.subsystems.apriltagvision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  class AprilTagVisionIOInputs implements LoggableInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public double[] demoFrame = new double[] {};
    public long fps = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamps", timestamps);
      table.put("FrameCount", frames.length);
      for (int i = 0; i < frames.length; i++) {
        table.put("Frame/" + i, frames[i]);
      }
      table.put("DemoFrame", demoFrame);
      table.put("Fps", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      timestamps = table.get("Timestamps", new double[] {0.0});
      int frameCount = table.get("FrameCount", 0);
      frames = new double[frameCount][];
      for (int i = 0; i < frameCount; i++) {
        frames[i] = table.get("Frame/" + i, new double[] {});
      }
      demoFrame = table.get("DemoFrame", new double[] {});
      fps = table.get("Fps", 0);
    }
  }

  default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
