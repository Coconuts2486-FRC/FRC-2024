package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public static Module frontRight = new Module(rotateFR, driveFR, encoderFR, new PIDController(rkp, rki, rkd),new PIDController(dkp, dki, dkd));
  public static Module frontLeft = new Module(rotateFL, driveFL, encoderFL, new PIDController(rkp, rki, rkd),new PIDController(dkp, dki, dkd));
  public static Module backRight = new Module(rotateBR, driveBR, encoderBR, new PIDController(rkp, rki, rkd),new PIDController(dkp, dki, dkd));
  public static Module backLeft = new Module(rotateBL, driveBL, encoderBL, new PIDController(rkp, rki, rkd),new PIDController(dkp, dki, dkd));

  public static Swerve swerve = new Swerve(backRight, backLeft, frontRight, frontLeft);

  public static Odometry odometry = new Odometry(backRight, backLeft, frontRight, frontLeft, swerve);
 

    // Shooter
    public static TalonSRX rightLauncher = new TalonSRX(15);
    public static TalonSRX leftLauncher = new TalonSRX(16);
    public static TalonFX launcherPivot = new TalonFX(17);

    //Intake
    public static TalonFX movementIntake = new TalonFX (18);
   public static TalonSRX intakeLeft = new TalonSRX(19);
   public static TalonSRX intakeRight = new TalonSRX (20);

  
    // Sensor
    public static DIOSensor limit1 = new DIOSensor(0,"limitSwitch");
    public static DIOSensor intakeStop = new DIOSensor(1, "lightSensor");
    public static DIOSensor intakeExtendStop = new DIOSensor(2, "limitSwitch");
    public static DIOSensor intakeCloseStop = new DIOSensor(3, "limitSwitch");

}
