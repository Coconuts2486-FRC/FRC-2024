package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;


/**
 * Represents a module of a robot, which consists of a direction motor, drive
 * motor, encoder, and PID controllers.
 */
@SuppressWarnings("removal")
public class Module {
  
    private TalonFX directionMotor;
      
    public TalonFX driveMotor;
      
    private CANCoder encoder;
    private PIDController angleController;
    private PIDController driveController;
    private double pi = Math.PI;

    // Internal Module Class
    
    public Module(
            int directionMotor,
            int driveMotor,
            int encoder,
            PIDController rController,
            PIDController dController) {
        this.directionMotor = new TalonFX(directionMotor, "drive");
        this.driveMotor = new TalonFX(driveMotor, "drive");
        this.encoder = new CANCoder(encoder, "drive");
        this.angleController = rController;
        this.driveController = dController;
    }

    /**
     * Initializes the module by setting the factory default,
     * inverting the direction motor, setting the neutral mode to brake,
     * and setting the drive motor to factory default and neutral mode to brake.
     */
      
    public void init() {
        directionMotor.configFactoryDefault();
        directionMotor.setInverted(true);
        directionMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(true);
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.configSelectedFeedbackSensor(
                TalonFXFeedbackDevice.IntegratedSensor,
                0,
                0);
    }

    /**
     * Disables the module by setting the neutral mode to coast for both the
     * direction and drive motors.
     */
    public void disable() {
        directionMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Enables the module by setting the neutral mode to brake for both the
     * direction and drive motors.
     */
    public double nearestAngle(double currentAngle, double targetAngle) {
        double direction = (targetAngle % (2 * pi) - (currentAngle % (2 * pi)));

        if (Math.abs(direction) > pi) {
            direction = -(Math.signum(direction) * (2 * pi)) + direction;
        }
        return direction;
    }

    /**
     * Drives the module based on the speed and angle.
     *
     * @param speed The speed of the module.
     * @param angle The angle of the module.
     */
      
    public void drive(double speed, double angle) {
        angleController.enableContinuousInput(-pi, pi);

        double currentAngle = (encoder.getAbsolutePosition() / 360) * (2 * pi);
        double setpoint = 0;

        double setpointAngle = nearestAngle(currentAngle, angle);
        double setpointAngleOpposite = nearestAngle(currentAngle, angle + pi);

        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleOpposite)) {
            setpoint = currentAngle + setpointAngle;
        } else {
            setpoint = currentAngle + setpointAngleOpposite;
            speed *= -1;
        }
        double optimizedAngle = angleController.calculate(currentAngle, setpoint);
        directionMotor.set(ControlMode.PercentOutput, optimizedAngle);
        double optimizedSpeed = driveController.calculate(speed);
        driveMotor.set(ControlMode.PercentOutput, optimizedSpeed);
        // SmartDashboard.putNumber("OptimizedAngle", optimizedAngle);
    }

    /**
     * Returns the speed of the module.
     * 
     * @return speed
     */
    public double getModuleSpeed() {
        return driveMotor.getSelectedSensorVelocity();
    }

    /**
     * Returns the angle of the module.
     * 
     * @return angle (radians)
     */
      
    public double getModuleAngle() {
        return (encoder.getAbsolutePosition() / 180) * pi;
    }

    /**
     * Returns the angle of the module in degrees.
     * 
     * @return angle (degrees)
     */
      
    public double getModuleAngleDeg() {
        return encoder.getAbsolutePosition();
    }

    /**
     * Initializes the module for autonomous by setting the angle controller to
     * continuous input and
     * setting the optimized angle to the current angle.
     *
     * @param angle The angle of the module.
     */
      
    public void autoInit(double angle) {
        // make pid continuous on (-pi, pi)
        angleController.enableContinuousInput(-pi, pi);

        // get the current position reading of the direction encoder
        double currentAngle = (encoder.getAbsolutePosition() * (2 * pi) / 360);
        // SmartDashboard.putNumber("Current Angle", currentAngle);
        double optimizedAngle = angleController.calculate(currentAngle, angle);
        directionMotor.set(ControlMode.PercentOutput, optimizedAngle);
        // directionMotor.set(ControlMode.Position,radiansToTicks(2*pi));

    }

    /**
     * Returns the optimized angle of the module.
     * 
     * @return optimized angle
     */
    public double getOptimizedAngle() {
        return angleController.calculate(getModuleAngleDeg(), 0);
    }

    /**
     * Returns the error of the module.
     * 
     * @return error code
     */
    public double getError() {
        return angleController.getPositionError();
    }

    /**
     * Returns the position of the module.
     * 
     * @return position
     */
    public double getPosition() {
        return directionMotor.getSelectedSensorPosition();
    }

    /**
     * Converts radians to ticks.
     *
     * @param radians The radians to convert.
     */
    public double radiansToTicks(double radians) {
        return radians * (44000 / (2 * pi));
    }

    /**
     * Initializes the module for autonomous by setting the angle controller to
     * continuous input and
     * setting the optimized angle to the current angle.
     *
     * @param angle The angle of the module.
     */
      
    public void autoInitDisabled(double angle) {
        // make pid continuous on (-pi, pi)
        angleController.enableContinuousInput(-pi, pi);

        // get the current position reading of the direction encoder
        double currentAngle = (encoder.getAbsolutePosition() * (2 * pi) / 360);
        // SmartDashboard.putNumber("Current Angle", currentAngle);
        double optimizedAngle = angleController.calculate(currentAngle, pi / 2);
        directionMotor.set(ControlMode.PercentOutput, optimizedAngle);
        // directionMotor.set(ControlMode.Position,radiansToTicks(2*pi));
    }

    /**
     * Returns the position of the module.
     */
      
    public double encoderPos() {
        return encoder.getAbsolutePosition();
    }
}
