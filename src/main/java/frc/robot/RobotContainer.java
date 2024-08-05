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
import frc.robot.commands.Auto.AutoIntakeCommand;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Auto.AutoShotCommand;
import frc.robot.commands.Drive.DriveCommands;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeRollersIOReal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.sma.SmaIntakeRollers;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Pivot pivot;
  private final Intake intake;
  private final IntakeRollers intakeRollers = new IntakeRollers(new IntakeRollersIOReal());
  private final Elevator elevator;
  private final DigitalInput lightStop = new DigitalInput(2);
  private final DigitalInput intakeStop = new DigitalInput(3);
  private final DigitalInput elevatorBottom = new DigitalInput(0); // change this
  public static final DigitalInput elevatorTop = new DigitalInput(1);
  private final SmaIntakeRollers smaIntakeRollers = new SmaIntakeRollers();

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController coDriver = new CommandXboxController(1);

  private final Trigger coDriverLeft = new Trigger(() -> coDriver.getLeftTriggerAxis() >= .1);
  private final Trigger coDriverRight = new Trigger(() -> coDriver.getRightTriggerAxis() >= .1);
  private final Trigger driverLeft = new Trigger(() -> driver.getLeftTriggerAxis() >= .1);
  private final Trigger driverRight = new Trigger(() -> driver.getRightTriggerAxis() >= .1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel
  // Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        // new Drive(
        // new GyroIOPigeon2(false),
        // new ModuleIOSparkMax(0),
        // new ModuleIOSparkMax(1),
        // new ModuleIOSparkMax(2),
        // new ModuleIOSparkMax(3));
        // flywheel = new Flywheel(new FlywheelIOSparkMax());
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        pivot = new Pivot(new PivotIOReal());
        intake = new Intake(new IntakeIOReal());
        elevator = new Elevator(new ElevatorIOReal());
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
        pivot = new Pivot(new PivotIOReal());
        flywheel = new Flywheel(new FlywheelIOSim());
        intake = new Intake(new IntakeIOReal());
        elevator = new Elevator(new ElevatorIOReal());
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
        break;
    }

    // Set up auto routines

    // this is example code. don't run. motor does wierd things
    /*
     * NamedCommands.registerCommand(
     * "Run Flywheel",
     * Commands.startEnd(
     * () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
     * flywheel)
     * .withTimeout(5.0));
     */

    // Can be added to auto path to tell robot to shoot during auto
    NamedCommands.registerCommand(
        "autoShoot", new AutoShotCommand(intakeRollers, flywheel, smaIntakeRollers).withTimeout(1));
    // Should Extend then activate rollers during auto... Maybe
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

    // idk path planner stuff
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
    // Drive Command
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> driver.getRightX()));
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Manual Intake
    intakeRollers.setDefaultCommand(
        ManualRollerCmd.manualRoller(
            intakeRollers, () -> driver.getRightTriggerAxis(), () -> driver.getLeftTriggerAxis()));
    // ** Normal Intake
    // - Rollers
    coDriver
        .y()
        .whileTrue(
            new IntakeRollerCommand(
                intakeRollers,
                () -> coDriver.getLeftTriggerAxis(),
                () -> coDriver.getRightTriggerAxis(),
                lightStop::get));
    // - Extend
    coDriver.y().whileTrue(new IntakeExtendCommand(intake, lightStop::get, intakeStop::get));
    // - Retract
    coDriver.y().whileFalse(new IntakeRetractCommand(intake, intakeStop::get));
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
    // Re-Zero Gyro
    driver.y().onTrue(Commands.runOnce(() -> drive.zero()));
    // ** Pivot Commands
    // > These three are the manual angle changer
    coDriver.povUp().whileTrue(new PivotChangerUpCommand());

    coDriver.povDown().whileTrue(new PivotChangerDownCommand());

    coDriver.povLeft().whileTrue(new PivotChangerResetCommand());
    // >
    // Go to 45
    // Adding the manual angle and the amp angle changer
    // Adding the manual angle and the amp angle changer
    coDriver
        .leftStick()
        .toggleOnTrue(
            new PivotCommand(pivot, () -> 45 + PivotChangerUpCommand.angler + AmpCommand.ampPivot));
    // Go to 60
    coDriver.back().whileTrue(new PivotCommand(pivot, () -> 60));
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
}
