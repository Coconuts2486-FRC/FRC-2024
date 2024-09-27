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

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Drive.DriveCommands;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(18);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  static Rotation2d saveYaw = new Rotation2d(0);

  private Rotation2d ppRotationOverride;

  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(

            // Sam, don't touch this.
            new PIDConstants(0.0015, 0.0005, 0.05),
            new PIDConstants(2, 0.0000, .5),
            MAX_LINEAR_SPEED,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);

    // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    poseEstimator.update(rawGyroRotation, modulePositions);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public void zero() {
    gyroIO.zero();
  }

  public void setAngle(double angle) {
    gyroIO.setAngle(angle);
  }

  public Rotation2d gyroAngles() {
    return gyroInputs.yawPosition;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  /**
   * Compute the field-centric YAW to the SPEAKER AprilTag, as seen by PhotonVision
   *
   * <p>Returns the field-centric YAW to the SPEAKER in degrees. To aim the robot at the speaker,
   * set the robot YAW equal to this value. If the speaker tag is not visible, this returns -999.9
   * degrees!
   *
   * <p>NOTE: This function assumes "Always Blue Origin" convention for YAW, meaning that when
   * alliance is BLUE, 0º is away from the alliance wall, and when alliance is RED, 180º is away
   * from the alliance wall. A head-on speaker shot has the robot facing away from the alliance wall
   * (i.e., the shooter is on the back of the robot).
   *
   * <p>NOTE: If we want the null result to return something other than -999.9, we can do that.
   */
  @AutoLogOutput(key = "Targeting/SpeakerYaw")
  public static Rotation2d getSpeakerYaw() {

    // No tag information, return default value
    if (AprilTagVision.speakerPose == null) {
      if (DriverStation.getAlliance().get() == Alliance.Red) { // :(
        return new Rotation2d(99999);
      } else {
        return new Rotation2d(99999);
      }
    }

    // The YAW to the speaker is computed from the X and Y position along the floor.

    Rotation2d yaw =
        new Rotation2d(
            Math.atan(AprilTagVision.robotPose.getY() / AprilTagVision.robotPose.getX()));

    // Rotate by 180º if on the RED alliance
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      yaw = yaw.plus(new Rotation2d(Math.PI));
    }

    // Return the YAW value as a Rotation2d object
    return yaw;
  }

  // This allows the robot to rotate to the gamepiece or speaker during auto
  // public Optional<Rotation2d> getRotationTargetOverride() {
  // System.out.println(Drive.getGamePieceYaw());

  // This checks if the commands are running we want running when turning to gamepiece
  // if (AutoIntakeCommandSlow.AutoRotos == 1 || AutoIntakeCommand.AutoRoto == 1) {
  // This makes sure the vision can see a Gamepiece so the robot doesn't spin in circles
  // violently
  // if (getGamePieceYaw() != null) {
  // saveYaw = Drive.getGamePieceYaw();
  // System.out.println("save" + saveYaw);
  // Return an optional containing the rotation override (this should be a field relative
  // rotation)

  // This gives the game piece yaw to rotation override which turns the robot towards the
  // piece
  //   Logger.recordOutput("AutoTargeting/GP_YAW", Drive.getGamePieceYaw());
  //   return Optional.of(Drive.getGamePieceYaw());
  // } else {
  //     // return an empty optional when we don't want to override the path's rotation
  //     return Optional.empty();
  //   }
  //   // Checks if commands are running that we want when turning towards speaker
  // } else if (AutoRegressedPivotCommand.AutoShoto == 1 || AutoSpinUpCommand.AutoShoto2 == 1)
  // {
  //   // makes sure that vision can see apriltags

  //         if (getSpeakerYaw() != null) {
  //            // makes robot turn towards speaker
  //           return Optional.of(Drive.getSpeakerYaw());
  //          } else {
  //           return Optional.empty();
  //          }
  //     }
  //  // } else {
  //    // return Optional.empty();
  //   }
  // }

  // /**
  //  * Compute the field-centric YAW to the GAMEPIECE, as seen by PiVision
  //  *
  //  * <p>Returns the field-centric YAW to the GAMEPIECE in degrees. To aim the robot at the game
  //  * piece, set the robot YAW equal to this value. If the game piece is not visible, this returns
  //  * -999.9 degrees!
  //  *
  //  * <p>NOTE: This function assumes "Always Blue Origin" convention for YAW, meaning that when
  //  * alliance is BLUE, 0º is away from the alliance wall, and when alliance is RED, 180º is away
  //  * from the alliance wall.
  //  *
  //  * <p>NOTE: If we want the null result to return something other than -999.9, we can do that.
  //  */
  @AutoLogOutput(key = "Targeting/GamePieceYaw")
  public static Rotation2d getGamePieceYaw() {

    // No tag information, return default value
    if (GamePieceVision.gamePieceRelativePose == null) {
      return new Rotation2d(Float.NaN);
    }

    // The YAW to the game piece is retrieved from PiVision relative to the robot
    Rotation2d gpYaw = new Rotation2d(Units.degreesToRadians(GamePieceVision.gamePieceRelYaw));

    // Add the game piece YAW to the current gyro value
    Rotation2d yaw = new Rotation2d(Units.degreesToRadians(DriveCommands.gyroYaw)).plus(gpYaw);

    // Return the YAW value as a Rotation2d object
    return yaw;
  }

  @AutoLogOutput(key = "Targeting/GamePiecePose")
  public static Pose2d getGamePiecePose() {

    return GamePieceVision.gamePieceRelativePose;
  }
}
