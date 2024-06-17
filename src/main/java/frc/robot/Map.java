package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Map {

    public static MotorOutputConfigs coast = new MotorOutputConfigs();
    public static MotorOutputConfigs brake = new MotorOutputConfigs();
    public static MotorOutputConfigs invertTrue = new MotorOutputConfigs();
    public static MotorOutputConfigs invertFalse = new MotorOutputConfigs();
   
    public static XboxController driver = new XboxController(0);
    public static Joystick coDriver = new Joystick(1);

    public static int encoderFL = 10;
    public static int encoderFR = 11;
    public static int encoderBL = 12;
    public static int encoderBR = 13;

    public static CANcoder FR = new CANcoder(11);
    public static CANcoder FL = new CANcoder(10);
    public static CANcoder BR = new CANcoder(13);
    public static CANcoder BL = new CANcoder(12);

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
            new PIDController(dkp, dki, dkd));
    public static Module frontLeft = new Module(
            rotateFL,
            driveFL,
            encoderFL,
            new PIDController(rkp, rki, rkd),
            new PIDController(dkp, dki, dkd));
    public static Module backRight = new Module(
            rotateBR,
            driveBR,
            encoderBR,
            new PIDController(rkp, rki, rkd),
            new PIDController(dkp, dki, dkd));
    public static Module backLeft = new Module(
            rotateBL,
            driveBL,
            encoderBL,
            new PIDController(rkp, rki, rkd),
            new PIDController(dkp, dki, dkd));

    public static Swerve swerve = new Swerve(
            backRight,
            backLeft,
            frontRight,
            frontLeft);


    // Shooter

    public static TalonFX bottomLauncher = new TalonFX(16);   
    public static TalonFX topLauncher = new TalonFX(15);
    

    // Elevator
    //public static TalonFX leftElevator = new TalonFX(21, "drive");
    public static TalonFX elevator = new TalonFX(22, "drive");
    // Intake
    public static TalonFX intakeExtend = new TalonFX(18);
    public static TalonSRX intakeBottom = new TalonSRX(19);
    public static TalonSRX intakeTop = new TalonSRX(20); 

    // Sensor
    public static DigitalInput elevatorBottom = new DigitalInput(0);
    public static DigitalInput elevatorTop = new DigitalInput(1);
    public static DigitalInput lightStop = new DigitalInput(2);
    public static DigitalInput intakeStop = new DigitalInput(3);
    public static DigitalInput pivotStop = new DigitalInput(4);
    // Misc
        public static SendableChooser red = new SendableChooser<>();
        public static SendableChooser selectedAuto = new SendableChooser<>();


        public static Odometry odometry = new Odometry(
            backRight,
            backLeft,
            frontRight,
            frontLeft,
            swerve);

            public static void init(){

        coast.withNeutralMode(NeutralModeValue.Coast);
        brake.withNeutralMode(NeutralModeValue.Brake);
        invertTrue.withInverted(InvertedValue.CounterClockwise_Positive);
        invertFalse.withInverted(InvertedValue.Clockwise_Positive);
    
            }
    
}
