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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.Logger;

public class GamePieceVisionIOPiVision implements GamePieceVisionIO {

  // The network table used for vision data.
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("vision");
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();

  // NOTE:: THESE SHOULD BE MODIFIED TO WORK WITH A FEED-FORWARD CONTROL!!!
  // The PID controller used for targeting a specific value.
  public static PIDController noteTargetPidClose = new PIDController(.55, 0, 0.00014);
  public static PIDController noteTargetPidFar = new PIDController(.3, 0, 0.00007);

  // The PID controller used for driving to a specific value.
  public static PIDController driveToPid = new PIDController(.03, 0.00, 0);

  // Actual function
  public GamePieceVisionIOPiVision() {

    // Important: The coordinate system used by PiVision is not the same as WPILib.  This function
    // does the translation into the WPILib coordinate system used elsewhere in this project.
    // Namely:
    //  WPILib.X = PiVision.Z
    //  WPILib.Y = PiVision.X
    //  WPILib.Z = PiVision.Y
    disconnectedAlert = new Alert("No data from PiVision", Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  public void updateInputs(GamePieceVisionIOInputs inputs) {

    double gamePieceX = Float.NaN;
    double gamePieceY = Float.NaN;
    double gamePieceZ = Float.NaN;
    double gamePieceAngle = Float.NaN;
    double gamePieceDist = Float.NaN;

    // Get observations
    // Game Piece X
    if (table.getEntry("gamepiece_robot_z").getDouble(0.0) != -999) {
      gamePieceX = table.getEntry("gamepiece_robot_z").getDouble(0.0);
    }
    // Game Piece Y
    if (table.getEntry("gamepiece_robot_x").getDouble(0.0) != -999) {
      gamePieceY = table.getEntry("gamepiece_robot_x").getDouble(0.0);
    }
    // Game Piece Z
    if (table.getEntry("gamepiece_robot_y").getDouble(0.0) != -999) {
      gamePieceZ = table.getEntry("gamepiece_robot_y").getDouble(0.0);
    }
    // Game Piece Angle
    if (table.getEntry("gamepiece_robot_angle").getDouble(0.0) != -999) {
      gamePieceAngle = table.getEntry("gamepiece_robot_angle").getDouble(0.0);
    }
    // Game Piece Distance
    if (table.getEntry("gamepiece_robot_dist").getDouble(0.0) != -999) {

      gamePieceDist = table.getEntry("gamepiece_robot_dist").getDouble(0.0);
    }
    // Log the information for posterity
    Logger.recordOutput("PiVision/GP_X", gamePieceX);
    Logger.recordOutput("PiVision/GP_Y", gamePieceY);
    Logger.recordOutput("PiVision/GP_Z", gamePieceZ);
    Logger.recordOutput("PiVision/GP_Angle", gamePieceAngle);
    Logger.recordOutput("PiVision/GP_Dist", gamePieceDist);

    // Pass all of the information back up the chain
    inputs.camname = "PiVision Gamepiece";
    inputs.latency = Float.NaN;
    inputs.timestamp = Float.NaN;

    inputs.gpX = gamePieceX;
    inputs.gpY = gamePieceX;
    inputs.gpZ = gamePieceX;
    inputs.gpAng = gamePieceX;
    inputs.gpDist = gamePieceX;
  }
}
