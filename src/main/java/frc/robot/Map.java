package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Map {
public static Joystick driver = new Joystick(0);

    public static int encoderFR = 10;
    public static int encoderFL = 11; 
    public static int encoderBR = 12;
    public static int encoderBL = 13;

    public static CANCoder FR = new CANCoder(10);
    public static CANCoder FL = new CANCoder(11);
    public static CANCoder BR = new CANCoder(12);
    public static CANCoder BL = new CANCoder(13);

    public static int driveFR = 2;
    public static int driveFL = 3; 
    public static int driveBR = 4;
    public static int driveBL = 5;

    public static int rotateFR = 6;
    public static int rotateFL = 7; 
    public static int rotateBR = 8;
    public static int rotateBL = 9;

    public static double kp = .63;
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
}
