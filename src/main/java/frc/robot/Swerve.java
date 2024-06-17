package frc.robot;



import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve {

    private static Module backRight;
    private static Module backLeft;
    private static Module frontRight;
    private static Module frontLeft;
    public static Pigeon2 gyro = new Pigeon2(14);

    private int realign = 0;
    private int reinit = 1;
   // private int robotCentric = 2;

    private double speedMultiplier = 1;

    public static double flOffset = (0);
  //  public static double flOffset = (3 * pi / 90);
    public static double frOffset = (0);
    public static double blOffset = (0);
    public static double brOffset = (0);

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
            backLeft.autoInit(blOffset);
            backRight.autoInit(brOffset);
            frontLeft.autoInit(flOffset);
            frontRight.autoInit(frOffset);
            gyro.getConfigurator().apply(new Pigeon2Configuration());
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
    public void disabled(boolean button) {
        backRight.disable(button);
        backLeft.disable(button);
        frontRight.disable(button);
        frontLeft.disable(button);
    }

    /**
     * Returns the robot angle in radians.
     * 
     * @return the robot angle in radians
     */
    public double getRobotAngle() {

        double robotAngle = gyro.getYaw().getValueAsDouble() * Math.PI / 180;

        return robotAngle;
    }


    /**
     * Wrapper around drive to accept velocity and angle
     * 
     * @param v Velocity
     * @param theta Angle (with respect to something...)
     * @param z The value of the swerve drive
     */

    public void drive_fc(double v, double theta, double z){
        double y = v * Math.cos(Math.toRadians(-theta));
        double x = -v * Math.sin(Math.toRadians(-theta));
        drive(x,y,z,false);
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
        double backRightSpeed = Math.sqrt((a * a) + (c * c));
        double backLeftSpeed = Math.sqrt((a * a) + (d * d));
        double frontRightSpeed = Math.sqrt((b * b) + (c * c));
        double frontLeftSpeed = Math.sqrt((b * b) + (d * d));

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
        double backRightAngle = Math.atan2(a, d);
        double backLeftAngle = Math.atan2(b0, d0);
        double frontRightAngle = Math.atan2(a0, c0);
        double frontLeftAngle = Math.atan2(b, c);

        backRight.drive(backRightSpeed, (backRightAngle ));
        backLeft.drive(backLeftSpeed, (backLeftAngle ));
        frontRight.drive(-frontRightSpeed, (frontRightAngle ));
        frontLeft.drive(-frontLeftSpeed, (frontLeftAngle ));
    }

    /**
     * Realigns the swerve drive to the field based on the button value.
     * 
     * @param button true if the swerve drive should be realigned, false otherwise
     */
    public void realignToField(boolean button) {
        if (button) {
            gyro.setYaw(180);
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
        reinit(Map.driver.getRawButton(reinit));
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
        SmartDashboard.putNumber("BR Angle", backRight.currentAngleRadians());
        SmartDashboard.putNumber("BR Optimized", backRight.getOptimizedAngle());
        SmartDashboard.putNumber("BL Angle", backLeft.currentAngleRadians());
        SmartDashboard.putNumber("BL Optimized", backLeft.getOptimizedAngle());
        SmartDashboard.putNumber("FR Angle", frontRight.currentAngleRadians());
        SmartDashboard.putNumber("Fr Optimized", frontRight.getOptimizedAngle());
        SmartDashboard.putNumber("FL Angle", frontLeft.currentAngleRadians());
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

        if (Map.driver.getRawButton(0)) {
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
        Map.frontLeft.driveMotor.setPosition(0);
        Map.frontRight.driveMotor.setPosition(0);
        Map.backLeft.driveMotor.setPosition(0);
        Map.backRight.driveMotor.setPosition(0);
    }
}
