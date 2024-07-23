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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.Drive.DriveCommands;
import frc.robot.commands.Elevator.ElevatorCommands;
import frc.robot.commands.Intake.IntakeExtendCommand;
import frc.robot.commands.Intake.IntakeRetractCommand;
import frc.robot.commands.Intake.IntakeRollerCommand;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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
  private final Trigger lightTrigger = new Trigger(lightStop::get);
  private final Trigger intakeLimitTrigger = new Trigger(intakeStop::get);
  private final Trigger elevatorBottomTrigger = new Trigger(elevatorBottom::get);

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController coDriver = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(false),
        //         new ModuleIOSparkMax(0),
        //         new ModuleIOSparkMax(1),
        //         new ModuleIOSparkMax(2),
        //         new ModuleIOSparkMax(3));
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
    /*NamedCommands.registerCommand(
    "Run Flywheel",
    Commands.startEnd(
            () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
        .withTimeout(5.0)); */

    // Can be added to auto path to tell robot to shoot during auto
    NamedCommands.registerCommand("autoShoot", new ShotCommand(intake, intakeRollers, flywheel));
    // Should Extend then activate rollers during auto... Maybe
    NamedCommands.registerCommand(
        "autoIntake",
        new AutoIntakeCommand(intakeRollers, intake, lightStop::get, intakeStop::get));

    // NamedCommands.registerCommand("autoIntake", new IntakeRetractCommand(intake,
    // intakeStop::get));

    // idk path planner stuff
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> driver.getRightX()));
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    elevator.setDefaultCommand(
        ElevatorCommands.manualElevator(
            elevator,
            () -> coDriver.getRightTriggerAxis(),
            () -> coDriver.getLeftTriggerAxis(),
            elevatorBottom::get,
            elevatorTop::get));

    coDriver
        .y()
        .whileTrue(
            new IntakeRollerCommand(
                intakeRollers,
                intake,
                () -> coDriver.getLeftTriggerAxis(),
                () -> coDriver.getRightTriggerAxis(),
                lightStop::get));

    coDriver.y().whileTrue(new IntakeExtendCommand(intake, lightStop::get, intakeStop::get));
    coDriver.y().whileFalse(new IntakeRetractCommand(intake, intakeStop::get));

    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driver.y().onTrue(Commands.runOnce(() -> drive.zero()));
    // Pivot Commands
    coDriver.leftStick().toggleOnTrue(new PivotCommand(pivot, () -> 45));
    coDriver.back().whileTrue(new PivotCommand(pivot, () -> 60));
    
    // shot command
    coDriver.rightBumper().whileTrue(new ShotCommand(intake, intakeRollers, flywheel));
    // climb command
    //   coDriver.b().whileTrue(new ClimbCommand(elevator, elevatorTop::get, -.90));
    //  coDriver.b().toggleOnFalse(new ClimbCommand(elevator, elevatorBottom::get, .40));
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
