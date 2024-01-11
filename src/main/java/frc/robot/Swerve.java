package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve {
    private Module backRight;
    private Module backLeft;
    private Module frontRight;
    private Module frontLeft;
    private static double pi = Math.PI;

    public static PigeonIMU gyro = new PigeonIMU(14);
    private XboxController driver = new XboxController(0);
    private int realign = 0;
    private int reinit = 1;
    private int robotCentric = 2;

    private double speedMultiplier = 1;

    public static double frOffset = (3 * pi / 90);
    public static double flOffset = (7 * pi / 90);
    public static double brOffset = (11.9 * pi / 30);
    public static double blOffset = (9 * pi / 90);

    private double x0 = 0.0;
    private double y0 = 0.0;

    double errorAngle = 0.0;
    double setpoint = 0.0;

    public Swerve(Module backRight, Module backLeft, Module frontRight, Module frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
    }

    public void init() {
        gyro.setYaw(0);
        backRight.init();
        backLeft.init();
        frontRight.init();
        frontLeft.init();
    }

    public void reinit(boolean reinit) {
        if (reinit) {
            gyro.setYaw(0);
            backRight.init();
            backLeft.init();
            frontRight.init();
            frontLeft.init();
            gyro.configFactoryDefault();
        }
    }

    public void disabled() {
        backRight.disable();
        backLeft.disable();
        frontRight.disable();
        frontLeft.disable();
    }

    public double getRobotAngle() {
        double[] ypr_deg = new double[3];

        gyro.getYawPitchRoll(ypr_deg);
        double robotAngle = ypr_deg[0] * Math.PI / 180;

        return robotAngle;
    }

    public void drive(double x, double y, double z) {
        double L = 17.5;
        double W = 17.5;
        double r = Math.sqrt((L * L) + (W * W));

        if (driver.getRawButton(robotCentric)) {
            x0 = x * speedMultiplier;
            y0 = y * speedMultiplier;
        } else {
            x0 = -y * Math.sin(getRobotAngle()) + x * Math.cos(getRobotAngle());
            y0 = y * Math.cos(getRobotAngle()) + x * Math.sin(getRobotAngle());
        }

        double a = x0 - z * (L / r);
        double b = x0 + z * (L / r);
        double c = y0 - z * (W / r);
        double d = y0 + z * (W / r);

        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        double maxBackSpeed = Math.max(backLeftSpeed, backRightSpeed);
        double maxFrontSpeed = Math.max(frontLeftSpeed, frontRightSpeed);
        double maxSpeed = Math.max(maxBackSpeed, maxFrontSpeed);

        if (maxSpeed > 1) {
            backRightSpeed = backRightSpeed / maxSpeed;
            backLeftSpeed = backLeftSpeed / maxSpeed;
            frontRightSpeed = frontRightSpeed / maxSpeed;
            frontLeftSpeed = frontLeftSpeed / maxSpeed;
        }

        double backRightAngle = Math.atan2(a, d);
        double backLeftAngle = Math.atan2(a, c);
        double frontRightAngle = Math.atan2(b, d);
        double frontLeftAngle = Math.atan2(b, c);

        backRight.drive(backRightSpeed, (backRightAngle + brOffset));
        backLeft.drive(backLeftSpeed, (backLeftAngle + blOffset));
        frontRight.drive(frontRightSpeed, (frontRightAngle + frOffset));
        frontLeft.drive(frontLeftSpeed, (frontLeftAngle + flOffset));
    }

    public void realignToField(int button) {
        if (driver.getRawButton(button)) {
            gyro.setYaw(0);
        }
    }

    public void run(double x, double y, double z, boolean track) {
        double twistAdjustment = 0;
        double twistDeadband = 0.175;
        double directionDeadband = 0.15;

        if (Math.abs(z) < twistDeadband || track) {
            z = 0.0;
        } else {
            z = (1 / (1 - twistDeadband)) * (z + -Math.signum(z) * twistDeadband);
        }
        if (Math.abs(x) < directionDeadband) {
            x = 0.0;
        } else {
            y = (1 / (1 - directionDeadband)) * (x + -Math.signum(x) * directionDeadband);
        }
        if (Math.abs(y) < directionDeadband) {
            y = 0.0;
        } else {
            y = (1 / (1 - directionDeadband)) * (y + -Math.signum(x) * directionDeadband);
        }
        drive(x, y, z + twistAdjustment);
        realignToField(realign);
        reinit(driver.getRawButton(reinit));
    }

    public void autoInit() {

        // backLeft.autoInit(0 * Math.PI);
        // backRight.autoInit(0 * Math.PI / 45);
        // frontLeft.autoInit(0 * Math.PI / 15);
        // frontRight.autoInit(0 * Math.PI / 60);
        backLeft.autoInit(blOffset);
        backRight.autoInit(brOffset);
        frontLeft.autoInit(flOffset);
        frontRight.autoInit(frOffset);

    }

    public void telemetry() {
        SmartDashboard.putNumber("BR Angle", backRight.getModuleAngle());
        SmartDashboard.putNumber("BR Optimized", backRight.getOptimizedAngle());
        SmartDashboard.putNumber("BL Angle", backLeft.getModuleAngle());
        SmartDashboard.putNumber("BL Optimized", backLeft.getOptimizedAngle());
        SmartDashboard.putNumber("FR Angle", frontRight.getModuleAngle());
        SmartDashboard.putNumber("Fr Optimized", frontRight.getOptimizedAngle());
        SmartDashboard.putNumber("FL Angle", frontLeft.getModuleAngle());
        SmartDashboard.putNumber("FL Optimized", frontLeft.getOptimizedAngle());

        SmartDashboard.putNumber("BR Error", backRight.getError());
        SmartDashboard.putNumber("BL Error", backLeft.getError());
        SmartDashboard.putNumber("FR Error", frontRight.getError());
        SmartDashboard.putNumber("FL Error", frontLeft.getError());

        SmartDashboard.putNumber("BR Encoder", backRight.encoderPos());
        SmartDashboard.putNumber("BL Encoder", backLeft.encoderPos());
        SmartDashboard.putNumber("FR Encoder", frontRight.encoderPos());
        SmartDashboard.putNumber("FL Encoder", frontLeft.encoderPos());

    }

    public void disabledPos() {
        SmartDashboard.putNumber("BR pos", backRight.getPosition());
        SmartDashboard.putNumber("BL pos", backLeft.getPosition());
        SmartDashboard.putNumber("FR pos", frontRight.getPosition());
        SmartDashboard.putNumber("FL pos", frontLeft.getPosition());
    }

    public void auto(double x, double y, double z) {
        double L = 17.5;
        double W = 17.5;
        double r = Math.sqrt((L * L) + (W * W));

        if (driver.getRawButton(robotCentric)) {
            x0 = x * speedMultiplier;
            y0 = y * speedMultiplier;
        } else {
            x0 = -y * Math.sin(getRobotAngle()) + x * Math.cos(getRobotAngle());
            y0 = y * Math.cos(getRobotAngle()) + x * Math.sin(getRobotAngle());
        }

        double a = x0 - z * (L / r);
        double b = x0 + z * (L / r);
        double c = y0 - z * (W / r);
        double d = y0 + z * (W / r);

        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        double maxBackSpeed = Math.max(backLeftSpeed, backRightSpeed);
        double maxFrontSpeed = Math.max(frontLeftSpeed, frontRightSpeed);
        double maxSpeed = Math.max(maxBackSpeed, maxFrontSpeed);

        if (maxSpeed > 1) {
            backRightSpeed = backRightSpeed / maxSpeed;
            backLeftSpeed = backLeftSpeed / maxSpeed;
            frontRightSpeed = frontRightSpeed / maxSpeed;
            frontLeftSpeed = frontLeftSpeed / maxSpeed;
        }

        double backRightAngle = Math.atan2(a, d);
        double backLeftAngle = Math.atan2(a, c);
        double frontRightAngle = Math.atan2(b, d);
        double frontLeftAngle = Math.atan2(b, c);

        backRight.drive(backRightSpeed, (backRightAngle + brOffset));
        backLeft.drive(backLeftSpeed, (backLeftAngle + blOffset));
        frontRight.drive(frontRightSpeed, (frontRightAngle + frOffset));
        frontLeft.drive(frontLeftSpeed, (frontLeftAngle + flOffset));
    }
}