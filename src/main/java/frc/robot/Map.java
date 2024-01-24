package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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

    public static double kp = .62;
    public static double ki = .0001;
    public static double kd = 0.;

    public static Module frontRight = new Module(rotateFR, driveFR, encoderFR, new PIDController(kp, ki, kd));
    public static Module frontLeft = new Module(rotateFL, driveFL, encoderFL, new PIDController(kp, ki, kd));
    public static Module backRight = new Module(rotateBR, driveBR, encoderBR, new PIDController(kp, ki, kd));
    public static Module backLeft = new Module(rotateBL, driveBL, encoderBL, new PIDController(kp, ki, kd));

    public static Swerve swerve = new Swerve(backRight, backLeft, frontRight, frontLeft);

    public static Odometry odometry = new Odometry(backRight, backLeft, frontRight, frontLeft, swerve);

      public static void encoderPos(){
      }
      //inisilazing launcher motors
        public static TalonSRX leftLauncher = new TalonSRX(14);
        public static TalonSRX rightLauncher = new TalonSRX(15);
        public static TalonFX launcherPivot = new TalonFX(16);
      

}
