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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.Auto.AutoIntakeCommand;
import frc.robot.commands.Auto.AutoShotCommand;
import frc.robot.commands.Auto.AutoSpinUpCommand;
import frc.robot.commands.Pivot.RegressedPivotCommand;
import frc.robot.commands.Drive.DriveCommands;
import frc.robot.commands.Drive.TargetNoteCommand;
import frc.robot.commands.Drive.TargetTagCommand;
import frc.robot.commands.Elevator.AmpCommand;
import frc.robot.commands.Elevator.ClimbCommand;
import frc.robot.commands.Elevator.ManualElevatorCommand;
import frc.robot.commands.Intake.IntakeExtendCommand;
import frc.robot.commands.Intake.IntakeRetractCommand;
import frc.robot.commands.Intake.IntakeRollerCommand;
import frc.robot.commands.Intake.ManualRollerCmd;
import frc.robot.commands.Pivot.PivotChangerDownCommand;
import frc.robot.commands.Pivot.PivotChangerResetCommand;
import frc.robot.commands.Pivot.PivotChangerUpCommand;
import frc.robot.commands.Pivot.PivotCommand;
import frc.robot.commands.ShotCommand;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhotonVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.gamepiecevision.GamePieceVision;
import frc.robot.subsystems.gamepiecevision.GamePieceVisionIO;
import frc.robot.subsystems.gamepiecevision.GamePieceVisionIOPiVision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeRollersIOReal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.sma.SmaIntakeRollers;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.OverrideSwitches;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems

  private final Pivot pivot;
  private final Intake intake;
  private final IntakeRollers intakeRollers = new IntakeRollers(new IntakeRollersIOReal());
  private final Elevator elevator;
  private final DigitalInput lightStop = new DigitalInput(2);
  private final DigitalInput intakeStop = new DigitalInput(3);
  private final DigitalInput elevatorBottom = new DigitalInput(0); // change this
  public static final DigitalInput elevatorTop = new DigitalInput(1);
  private final SmaIntakeRollers smaIntakeRollers = new SmaIntakeRollers();

  private final Drive drive;
  private final Flywheel flywheel;
  private final AprilTagVision aprilTagVision;
  private final GamePieceVision gamePieceVision;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController coDriver = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(2);

  private final Trigger coDriverLeft = new Trigger(() -> coDriver.getLeftTriggerAxis() >= .1);
  private final Trigger coDriverRight = new Trigger(() -> coDriver.getRightTriggerAxis() >= .1);
  private final Trigger driverLeft = new Trigger(() -> driver.getLeftTriggerAxis() >= .1);
  private final Trigger driverRight = new Trigger(() -> driver.getRightTriggerAxis() >= .1);

  // Override Switches
  private final Trigger aprilTagsTargetOnly = overrides.operatorSwitch(0);

  // Alerts
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel
  // Speed", 1500.0);

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getAprilTagLayoutType() {
    return FieldConstants.defaultAprilTagType;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        pivot = new Pivot(new PivotIOReal());
        intake = new Intake(new IntakeIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        aprilTagVision =
            new AprilTagVision(
                this::getAprilTagLayoutType,
                new AprilTagVisionIOPhotonVision(this::getAprilTagLayoutType, "Photon_BW1"),
                new AprilTagVisionIOPhotonVision(this::getAprilTagLayoutType, "Photon_BW2"));
        gamePieceVision = new GamePieceVision(new GamePieceVisionIOPiVision());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        pivot = new Pivot(new PivotIOReal());
        intake = new Intake(new IntakeIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        aprilTagVision = new AprilTagVision(this::getAprilTagLayoutType);
        gamePieceVision = new GamePieceVision();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        pivot = new Pivot(new PivotIOReal());
        intake = new Intake(new IntakeIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        aprilTagVision =
            new AprilTagVision(
                this::getAprilTagLayoutType, new AprilTagVisionIO() {}, new AprilTagVisionIO() {});
        gamePieceVision = new GamePieceVision(new GamePieceVisionIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "autoShoot",
        new AutoShotCommand(intakeRollers, flywheel, smaIntakeRollers).withTimeout(0.4));
    NamedCommands.registerCommand(
        "autoShootFull",
        new AutoShotCommand(intakeRollers, flywheel, smaIntakeRollers).withTimeout(0.8));
    NamedCommands.registerCommand("autoSpinUp", new AutoSpinUpCommand(flywheel));
    NamedCommands.registerCommand(
        "autoIntake",
        new AutoIntakeCommand(
            intakeRollers,
            smaIntakeRollers,
            intake,
            lightStop::get,
            intakeStop::get,
            pivot,
            () -> 45));
    NamedCommands.registerCommand("PivotAmp45", new PivotCommand(pivot, () -> 60));
    NamedCommands.registerCommand("PivotAmp23.6", new PivotCommand(pivot, () -> 23.52));
    NamedCommands.registerCommand("PivotAmp23", new PivotCommand(pivot, () -> 23));
    NamedCommands.registerCommand("PivotAmp25.5", new PivotCommand(pivot, () -> 25.5));
    NamedCommands.registerCommand("PivotRegressed", new RegressedPivotCommand(pivot, () -> 0));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getRightY(),
            () -> -driver.getRightX(),
            () -> -driver.getLeftX(),
            lightStop::get));
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    coDriver.start().whileTrue(new TargetNoteCommand());

    // Manual Intake
    intakeRollers.setDefaultCommand(
        ManualRollerCmd.manualRoller(
            intakeRollers,
            () -> driver.getRightTriggerAxis(),
            () -> driver.getLeftTriggerAxis(),
            lightStop::get));

    coDriver.leftBumper().whileTrue(new TargetTagCommand());

    // ** Normal Intake
    // - Rollers
    driver
        .rightBumper()
        .whileTrue(
            new IntakeRollerCommand(
                intakeRollers,
                () -> coDriver.getLeftTriggerAxis(),
                () -> coDriver.getRightTriggerAxis(),
                lightStop::get));
    // - Extend
    driver
        .rightBumper()
        .whileTrue(new IntakeExtendCommand(intake, lightStop::get, intakeStop::get));
    // - Retract
    driver.rightBumper().whileFalse(new IntakeRetractCommand(intake, intakeStop::get));
    // **
    // I Actually Don't know
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // controller
    //     .a()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));

    // Re-Zero Gyro
    driver.y().onTrue(Commands.runOnce(() -> drive.zero()));

    // ############################################/
    // Re-Zero Gyro :: DOUBLE-UP BECAUSE driver.a() causes a funny odometry reset, but is not
    // defined here!
    driver.a().onTrue(Commands.runOnce(() -> drive.zero()));
    // ############################################/

    // ** Pivot Commands
    // > These three are the manual angle changer
    coDriver.povUp().whileTrue(new PivotChangerUpCommand());

    coDriver.povDown().whileTrue(new PivotChangerDownCommand());

    coDriver.povLeft().whileTrue(new PivotChangerResetCommand());
    // >
    // Go to 45
    // Adding the manual angle and the amp angle changer

    coDriver
        .leftStick()
        .toggleOnTrue(
            new PivotCommand(pivot, () -> 45 + PivotChangerUpCommand.angler + AmpCommand.ampPivot));
    // Go to 60
    coDriver.back().whileTrue(new PivotCommand(pivot, () -> 60));

    coDriver
        .leftBumper()
        .whileTrue(
            new RegressedPivotCommand(pivot, () -> PivotChangerUpCommand.angler)
                .andThen(
                    new PivotCommand(
                        pivot, () -> 45 + PivotChangerUpCommand.angler + AmpCommand.ampPivot)));

    // **
    // Shot
    coDriver.rightBumper().whileTrue(new ShotCommand(intakeRollers, flywheel));
    //   Amp command
    coDriver
        .b()
        .toggleOnTrue(new AmpCommand(intakeRollers, elevator, lightStop::get, elevatorTop::get));
    coDriver
        .b()
        .toggleOnFalse(
            new ClimbCommand(
                    elevator,
                    elevatorBottom::get,
                    () -> coDriver.getRightTriggerAxis(),
                    () -> coDriver.getLeftTriggerAxis(),
                    .6,
                    true)
                .until(elevatorBottom::get));
    // climb command
    coDriverRight.whileTrue(
        new ManualElevatorCommand(
            elevator,
            elevatorTop::get,
            elevatorBottom::get,
            () -> coDriver.getRightTriggerAxis(),
            () -> coDriver.getLeftTriggerAxis()));

    coDriverLeft.whileTrue(
        new ManualElevatorCommand(
            elevator,
            elevatorTop::get,
            elevatorBottom::get,
            () -> coDriver.getRightTriggerAxis(),
            () -> coDriver.getLeftTriggerAxis()));

    coDriver
        .a()
        .toggleOnTrue(
            new ClimbCommand(
                elevator,
                elevatorTop::get,
                () -> coDriver.getRightTriggerAxis(),
                () -> coDriver.getLeftTriggerAxis(),
                -.90,
                false));
    coDriver
        .a()
        .toggleOnFalse(
            new ClimbCommand(
                    elevator,
                    elevatorBottom::get,
                    () -> coDriver.getRightTriggerAxis(),
                    () -> coDriver.getLeftTriggerAxis(),
                    .60,
                    true)
                .until(elevatorBottom::get));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive = getAprilTagLayoutType() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use (" + getAprilTagLayoutType().toString() + ").");
    }
  }
}
