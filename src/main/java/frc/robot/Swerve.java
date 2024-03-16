package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Swerve {

    private static Module backRight;
    private static Module backLeft;
    private static Module frontRight;
    private static Module frontLeft;
    private static double pi = Math.PI;

    public static PigeonIMU gyro = new PigeonIMU(14);
    public static PigeonIMU gyro2 = new PigeonIMU(30);
    private XboxController driver = new XboxController(0);
    private int realign = 0;
    private int reinit = 1;
   // private int robotCentric = 2;

    private double speedMultiplier = 1;

    public static double flOffset = (3 * pi / 90);
    public static double frOffset = (7 * pi / 90);
    public static double blOffset = (11.9 * pi / 30);
    public static double brOffset = (9 * pi / 90);

    private double x0 = 0.0;
    private double y0 = 0.0;

    double errorAngle = 0.0;
    double setpoint = 0.0;

    public Swerve(
            Module backRight,
            Module backLeft,
            Module frontRight,
            Module frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
    }

    public void init() {
        gyro.setYaw(180);
        gyro2.setYaw(180);
        backRight.init();
        backLeft.init();
        frontRight.init();
        frontLeft.init();

    }

    /**
     * Reinitializes the swerve drive by resetting the gyro yaw, and setting the
     * module angles to 0.
     * 
     * @param reinit true if the swerve drive should be reinitialized, false
     *               otherwise
     */
    public static void reinit(boolean reinit) {
        if (reinit) {
            gyro.setYaw(180);
            gyro2.setYaw(180);
            backLeft.autoInit(blOffset);
            backRight.autoInit(brOffset);
            frontLeft.autoInit(flOffset);
            frontRight.autoInit(frOffset);
            gyro.configFactoryDefault();
        }

    }

    /**
     * Initialize the modules
     */
    public static void modInit() {
        backRight.init();
        backLeft.init();
        frontRight.init();
        frontLeft.init();
    }

    /**
     * Disables the swerve drive by setting the neutral mode to coast for all the
     * modules.
     */
    public void disabled() {
        backRight.disable();
        backLeft.disable();
        frontRight.disable();
        frontLeft.disable();
    }

    /**
     * Returns the robot angle in radians.
     * 
     * @return the robot angle in radians
     */
    public double getRobotAngle() {

        double robotAngle = gyro.getYaw() * Math.PI / 180;

        return robotAngle;
    }

    /**
     * Drives the swerve drive based on the x, y, and z values.
     * 
     * @param x The x value of the swerve drive.
     * @param y The y value of the swerve drive.
     * @param z The z value of the swerve drive.
     */
    public void drive(double x, double y, double z, boolean robotCentric) {
        double L = 17.5; // length of wheelbase
        double W = 17.5; // width of wheelbase
        double r = Math.hypot(L, W);

        // This button is not being used in 2024
        if (robotCentric) {
            x0 = x * speedMultiplier;
            y0 = y * speedMultiplier;
        } 
        // This is the usual case for 2024
        else {
            double robotAngle = getRobotAngle();
            x0 = -y * Math.sin(robotAngle) + x * Math.cos(robotAngle);
            y0 = y * Math.cos(robotAngle) + x * Math.sin(robotAngle);
        }

        // Swerve drive awesome maths!!
        double a = x0 - z * (L / r);
        double b = x0 + z * (L / r);
        double c = y0 - z * (W / r);
        double d = y0 + z * (W / r);

        double a0 = -x0 - z * (L / r);
        double b0 = -x0 + z * (L / r);
        double c0 = -y0 - z * (W / r);
        double d0 = -y0 + z * (W / r);

        // double backRightSpeed = Math.sqrt((a * a) + (d * d));
        // double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        // double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        // double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        // NOTE: can change sqrt -> hypot as desired
        double backRightSpeed = Math.sqrt((a0 * a0) + (c0 * c0));
        double backLeftSpeed = Math.sqrt((a * a) + (d * d));
        double frontRightSpeed = Math.sqrt((b * b) + (c * c));
        double frontLeftSpeed = Math.sqrt((b0 * b0) + (d0 * d0));

        double maxBackSpeed = Math.max(backLeftSpeed, backRightSpeed);
        double maxFrontSpeed = Math.max(frontLeftSpeed, frontRightSpeed);
        double maxSpeed = Math.max(maxBackSpeed, maxFrontSpeed);

        if (maxSpeed > 1) {
            backRightSpeed = backRightSpeed / maxSpeed;
            backLeftSpeed = backLeftSpeed / maxSpeed;
            frontRightSpeed = frontRightSpeed / maxSpeed;
            frontLeftSpeed = frontLeftSpeed / maxSpeed;
        }

        // double backRightAngle = Math.atan2(a, d);
        // double backLeftAngle = Math.atan2(a, c);
        // double frontRightAngle = Math.atan2(b, d);
        // double frontLeftAngle = Math.atan2(b, c);
        double backRightAngle = Math.atan2(a0, c0);
        double backLeftAngle = Math.atan2(a, d);
        double frontRightAngle = Math.atan2(b, c);
        double frontLeftAngle = Math.atan2(b0, d0);

        backRight.drive(backRightSpeed, (backRightAngle + brOffset));
        backLeft.drive(backLeftSpeed, (backLeftAngle + blOffset));
        frontRight.drive(frontRightSpeed, (frontRightAngle + frOffset));
        frontLeft.drive(frontLeftSpeed, (frontLeftAngle + flOffset));
    }

    /**
     * Realigns the swerve drive to the field based on the button value.
     * 
     * @param button true if the swerve drive should be realigned, false otherwise
     */
    public void realignToField(boolean button) {
        if (button) {
            gyro.setYaw(180);
            gyro2.setYaw(180);
        }
    }
    /**
     * Runs the swerve drive based on the x, y, and z values and the track value.
     * 
     * @param x     The x value of the swerve drive.
     * @param y     The y value of the swerve drive.
     * @param z     The z value of the swerve drive.
     * @param track true if the swerve drive should track, false otherwise
     */
    public void run(double x, double y, double z, boolean track) {
        double twistAdjustment = 0;
        double twistDeadband = 0.275;
        double directionDeadband = 0.2;

        if (Math.abs(z) < twistDeadband || track) {
            z = 0.0;
        } else {
            z = (1 / (1 - twistDeadband)) * (z + -Math.signum(z) * twistDeadband);
        }
        if (Math.abs(x) < directionDeadband) {
            x = 0.0;
        } else {
            y = (1 / (1 - directionDeadband)) *
                    (x + -Math.signum(x) * directionDeadband);
        }
        if (Math.abs(y) < directionDeadband) {
            y = 0.0;
        } else {
            y = (1 / (1 - directionDeadband)) *
                    (y + -Math.signum(x) * directionDeadband);
        }
        drive(x, y, z + twistAdjustment,false);
        realignToField(Map.driver.getRawButton(realign));
        reinit(driver.getRawButton(reinit));
    }

    /**
     * Initializes the swerve drive based on the angle.
     * 
     * @param angle The angle of the swerve drive
     */
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

    /**
     * Telemetries the swerve drive.
     */
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

    /**
     * Disables the position of the swerve drive.
     */
    public void disabledPos() {
        SmartDashboard.putNumber("BR pos", backRight.getPosition());
        SmartDashboard.putNumber("BL pos", backLeft.getPosition());
        SmartDashboard.putNumber("FR pos", frontRight.getPosition());
        SmartDashboard.putNumber("FL pos", frontLeft.getPosition());
    }

    /**
     * Drives the swerve drive based on the x, y, and z values.
     * 
     * @param x The x value of the swerve drive.
     * @param y The y value of the swerve drive.
     * @param z The z value of the swerve drive.
     */
    public void auto(double x, double y, double z) {
        double L = 17.5;
        double W = 17.5;
        double r = Math.sqrt((L * L) + (W * W));

        if (driver.getRawButton(0)) {
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

    /**
     * Realigns the swerve drive to the field based on the button value.
     * 
     * @param button true if the swerve drive should be realigned, false otherwise
     */
    public static void reZeroPosition() {
        Map.frontLeft.driveMotor.setSelectedSensorPosition(0);
        Map.frontRight.driveMotor.setSelectedSensorPosition(0);
        Map.backLeft.driveMotor.setSelectedSensorPosition(0);
        Map.backRight.driveMotor.setSelectedSensorPosition(0);
    }
}
