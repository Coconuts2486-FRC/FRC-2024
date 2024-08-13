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

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO.AprilTagVisionIOInputs;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Vision subsystem for AprilTag vision (built upon a virtual subsystem) */
@ExtensionMethod({GeomUtil.class})
public class AprilTagVision extends VirtualSubsystem {

  private static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", -(1.0 / 50.0));
  private static final double demoTagPosePersistenceSecs = 0.5;

  // Tag sources and layout
  private final Supplier<AprilTagLayoutType> aprilTagTypeSupplier;
  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  // Bookkeeping for last frame exposure and last tag detection
  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  // Something to do with demo?
  private Pose3d demoTagPose = null;
  private double lastDemoTagPoseTimestamp = 0.0;
  private static Pose3d robotPose = null;
  public static Pose3d speakerPose = null;
  private static Translation3d speakerTranslation = null;
  private static Rotation3d robotRotation = null;
  private static Pose3d thisSpeakerPose = null;

  // Class method definition, including inputs
  public AprilTagVision(Supplier<AprilTagLayoutType> aprilTagTypeSupplier, AprilTagVisionIO... io) {
    this.aprilTagTypeSupplier = aprilTagTypeSupplier;
    this.io = io;
    inputs = new AprilTagVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AprilTagVisionIOInputs();
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      // NOTE: This line causes the RIO to overrun RAM
      // Logger.processInputs("AprilTagVision/Inst" + i, inputs[i]);
    }

    // Loop over cameras
    List<Pose2d> allSpeakerPoses = new ArrayList<>();
    List<Pose3d> allSpeakerPoses3d = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      var timestamp = inputs[instanceIndex].timestamp;
      var latency = inputs[instanceIndex].latency;
      var targets = inputs[instanceIndex].targets;

      // Exit if no targets
      if (targets == null) {
        continue;
      }

      // Loop over found targets
      for (PhotonTrackedTarget target : targets) {

        // Get the speaker of interest for this alliance
        if ((target.getFiducialId() == 4 && DriverStation.getAlliance().get() == Alliance.Red)
            || (target.getFiducialId() == 7
                && DriverStation.getAlliance().get() == Alliance.Blue)) {

          // Camera pose is "WHERE IS CAMERA AS SEEN FROM TAG"; invert to get tag from camera
          Pose3d cameraPose = GeomUtil.toPose3d(target.getBestCameraToTarget().inverse());
          robotPose = cameraPose.transformBy(cameraPoses[instanceIndex].toTransform3d().inverse());

          // Get the speaker pose from the point of view of the robot
          robotRotation = robotPose.getRotation();
          speakerTranslation =
              robotPose.getTranslation().unaryMinus().rotateBy(robotRotation.unaryMinus());
          thisSpeakerPose = new Pose3d(speakerTranslation, new Rotation3d());

          allSpeakerPoses3d.add(thisSpeakerPose);
          allSpeakerPoses.add(thisSpeakerPose.toPose2d());

          // Log the relevant information to AdvantageKit
          Logger.recordOutput("PhotonVision/Speaker_" + inputs[instanceIndex].camname, speakerPose);
          Logger.recordOutput(
              "PhotonVision/Robot2d_" + inputs[instanceIndex].camname,
              robotPose.getRotation().toRotation2d());
        }
      }
    }
    // Set output value based on number of tags seen
    switch ((int) allSpeakerPoses3d.size()) {
      case 0:
        // If no speaker tags, return a null Pose3d
        speakerPose = null;
        break;
      case 1:
        // One tag seen, return it
        speakerPose = allSpeakerPoses3d.get(0);
        break;
      default:
        // Otherwise, compute the average!
        Translation3d trans0 = allSpeakerPoses3d.get(0).getTranslation();
        Translation3d trans1 = allSpeakerPoses3d.get(1).getTranslation();
        speakerPose =
            new Pose3d(
                (trans0.getX() + trans1.getX()) / 2.,
                (trans0.getY() + trans1.getY()) / 2.,
                (trans0.getZ() + trans1.getZ()) / 2.,
                new Rotation3d());
    }
  }
}
