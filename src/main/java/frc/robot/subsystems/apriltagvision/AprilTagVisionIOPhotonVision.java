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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Alert;
import frc.robot.util.GeomUtil;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionIOPhotonVision implements AprilTagVisionIO {

  private final Supplier<AprilTagLayoutType> aprilTagTypeSupplier;
  private AprilTagLayoutType lastAprilTagType = null;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();
  public final PhotonCamera camera;
  public final String camname;
  private PhotonPoseEstimator photonPoseEstimator = null;
  public final int instanceIndex;
  public final Drive drive;

  public AprilTagVisionIOPhotonVision(
      Supplier<AprilTagLayoutType> aprilTagTypeSupplier,
      String camname,
      int instanceIndex,
      Drive drive) {
    this.aprilTagTypeSupplier = aprilTagTypeSupplier;
    this.camname = camname;
    this.camera = new PhotonCamera(camname);
    this.instanceIndex = instanceIndex;
    this.drive = drive;

    // The field from AprilTagFields will be different depending on the game.
    AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // Construct PhotonPoseEstimator
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            this.camera,
            GeomUtil.toTransform3d(cameraPoses[instanceIndex]));

    disconnectedAlert = new Alert("No data from \"" + camname + "\"", Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    // Get observations
    var result = camera.getLatestResult();

    // Log the entire result for each camera to AdvantageKit
    Logger.recordOutput("PhotonVision/" + camname, result); // result.getLatencyMillis());

    // Put the relevant information into the `inputs`
    inputs.camname = camname;
    inputs.latency = result.getLatencyMillis();
    inputs.timestamp = result.getTimestampSeconds();

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      inputs.targets = targets;
      for (PhotonTrackedTarget target : targets) {
        // Get information from target.
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

        String logTag = "PhotonVision/Tag" + Integer.toString(targetID);
        Logger.recordOutput(logTag + "/PoseAmbiguity", poseAmbiguity);
        Logger.recordOutput(logTag + "/BestCameraToTarget", bestCameraToTarget);
        Logger.recordOutput(logTag + "/AlternateCameraToTarget", alternateCameraToTarget);
      }
      Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(drive.getPose());
      Logger.recordOutput(
          "PhotonVision/" + camname + "_pose",
          estimatedPose.orElse(new EstimatedRobotPose(new Pose3d(), 0.0, targets, null))
              .estimatedPose);

    } else {
      inputs.targets = null;
    }
  }
}
