package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*  Conroller bindings:
 *    ===Driver===
 *  Right stick x & y: strafe
 *  Left stick x: rotate
 *  Left stick y:
 *  Left stick button:
 *  Right stick button:
 *  A: Re-Zero
 *  B:
 *  X:
 *  Y:
 *  Right Bumper: intake out + intake + target
 *  Left Bumper: intake in
 *  Right trigger:
 *  Left Trigger: outtake
 *  POV:
 *  Back:
 *  Start:
 *
 *    ===CoDriver==
 *  Right stick x
 *  Right stick y
 *  Left stick x:
 *  Left stick y: manual elevator control
 *  Left stick button:
 *  Right stick button:
 *  A: Elevator position 0
 *  B: Elevator position amp score
 *  X: Elevator manual mode on
 *  Y: Elevator climb
 *  Right Bumper: shoot
 *  Left Bumper: Aim/spin up
 *  Right trigger: manual aim
 *  Left Trigger: manual aim
 *  POV + Start: Swap controllers
 *  Back:
 */

/**
 * This class represents the map of the robot.
 * It provides the controller bindings and the motor and sensor ports.
 */
public class Map {

  public static XboxController driver = new XboxController(0);
  public static Joystick coDriver = new Joystick(1);

  public static int encoderFL = 10;
  public static int encoderFR = 11;
  public static int encoderBL = 12;
  public static int encoderBR = 13;

  public static CANCoder FR = new CANCoder(10);
  public static CANCoder FL = new CANCoder(11);
  public static CANCoder BR = new CANCoder(12);
  public static CANCoder BL = new CANCoder(13);

  public static int driveFL = 2;
  public static int driveFR = 3;
  public static int driveBL = 4;
  public static int driveBR = 5;

  public static int rotateFL = 6;
  public static int rotateFR = 7;
  public static int rotateBL = 8;
  public static int rotateBR = 9;

  public static double rkp = .617;
  public static double rki = .0002;
  public static double rkd = 0.;

  public static double dkp = 1.0;
  public static double dki = .000;
  public static double dkd = 0.0001;

  public static Module frontRight = new Module(
    rotateFR,
    driveFR,
    encoderFR,
    new PIDController(rkp, rki, rkd),
    new PIDController(dkp, dki, dkd)
  );
  public static Module frontLeft = new Module(
    rotateFL,
    driveFL,
    encoderFL,
    new PIDController(rkp, rki, rkd),
    new PIDController(dkp, dki, dkd)
  );
  public static Module backRight = new Module(
    rotateBR,
    driveBR,
    encoderBR,
    new PIDController(rkp, rki, rkd),
    new PIDController(dkp, dki, dkd)
  );
  public static Module backLeft = new Module(
    rotateBL,
    driveBL,
    encoderBL,
    new PIDController(rkp, rki, rkd),
    new PIDController(dkp, dki, dkd)
  );

  public static Swerve swerve = new Swerve(
    backRight,
    backLeft,
    frontRight,
    frontLeft
  );

  public static Odometry odometry = new Odometry(
    backRight,
    backLeft,
    frontRight,
    frontLeft,
    swerve
  );

  // Shooter
  public static TalonSRX rightLauncher = new TalonSRX(15);
  public static TalonSRX leftLauncher = new TalonSRX(16);
 public static int launcherPivot = 17;

  // Elevator
  public static TalonFX leftElevator = new TalonFX(21,"drive");
  public static TalonFX rightElevator = new TalonFX(22,"drive");

  // Intake
  public static TalonFX movementIntake = new TalonFX(18);
  public static TalonSRX intakeLeft = new TalonSRX(19);
  public static TalonSRX intakeRight = new TalonSRX(20);

 
    // Sensor
    public static DigitalInput elevatorBottom = new DigitalInput(0);
    public static DigitalInput elevatorTop = new DigitalInput(1);
    public static DigitalInput lightStop = new DigitalInput(2);
    public static DigitalInput intakeStop = new DigitalInput(3);
    public static DigitalInput pivotTop = new DigitalInput(4);
   // public static DIOSensor lightStop = new DIOSensor(2, "Normal-Light");
    // public static DIOSensor intakeStop = new DIOSensor(3, "Normal");
    // public static DIOSensor pivotTop = new DIOSensor(4, "Strange");
    //public static DIOSensor elevatorTop = new DIOSensor(1, "Normal");
    //Misc
    public static SendableChooser red = new SendableChooser<>();
        public static SendableChooser selectedAuto = new SendableChooser<>();

}
