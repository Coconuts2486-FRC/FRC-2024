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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class AprilTagVisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.005;
  public static final double thetaStdDevCoefficient = 0.01;

  public static final double[] stdDevFactors =
      switch (Constants.getRobot()) {
        case COMPBOT -> new double[] {1.0, 0.6, 1.0, 1.2};
        case DEVBOT -> new double[] {1.0, 1.0};
        default -> new double[] {};
      };

  public static final Pose3d[] cameraPoses =
      switch (Constants.getRobot()) {
        case COMPBOT -> new Pose3d[] {
          new Pose3d(
              Units.inchesToMeters(8.875),
              Units.inchesToMeters(10.5),
              Units.inchesToMeters(8.25),
              new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)))),
          new Pose3d(
              Units.inchesToMeters(3.25),
              Units.inchesToMeters(5.0),
              Units.inchesToMeters(6.4),
              new Rotation3d(0.0, Units.degreesToRadians(-16.875), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-4.709)))),
          new Pose3d(
              Units.inchesToMeters(8.875),
              Units.inchesToMeters(-10.5),
              Units.inchesToMeters(8.25),
              new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-30.0)))),
          new Pose3d(
              Units.inchesToMeters(-16.0),
              Units.inchesToMeters(-12.0),
              Units.inchesToMeters(8.5),
              new Rotation3d(0.0, Units.degreesToRadians(-33.75), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(176.386))))
        };
        case DEVBOT -> new Pose3d[] {
          new Pose3d(
              Units.inchesToMeters(9.735),
              Units.inchesToMeters(9.974),
              Units.inchesToMeters(8.837),
              new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)))),
          new Pose3d(
              Units.inchesToMeters(9.735),
              Units.inchesToMeters(-9.974),
              Units.inchesToMeters(8.837),
              new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-30.0))))
        };
        default -> new Pose3d[] {};
      };

  public static final String[] instanceNames =
      switch (Constants.getRobot()) {
        case COMPBOT -> new String[] {"northstar_0", "northstar_1", "northstar_2", "northstar_3"};
        case DEVBOT -> new String[] {"northstar_0", "northstar_1"};
        default -> new String[] {};
      };

  public static final String[] cameraIds =
      switch (Constants.getRobot()) {
        case COMPBOT -> new String[] {
          "/dev/v4l/by-path/platform-fc800000.usb-usb-0:1:1.0-video-index0",
          "/dev/v4l/by-path/platform-fc880000.usb-usb-0:1:1.0-video-index0",
          "/dev/v4l/by-path/platform-fc800000.usb-usb-0:1:1.0-video-index0",
          "/dev/v4l/by-path/platform-fc880000.usb-usb-0:1:1.0-video-index0"
        };
        case DEVBOT -> new String[] {
          "/dev/v4l/by-path/platform-fc800000.usb-usb-0:1:1.0-video-index0",
          "/dev/v4l/by-path/platform-fc880000.usb-usb-0:1:1.0-video-index0"
        };
        default -> new String[] {};
      };
}
