package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module {
    private TalonFX directionMotor;
    public TalonFX driveMotor;
    private CANCoder encoder;
    private PIDController angleController;
    private double pi = Math.PI;

    public Module(int directionMotor, int driveMotor, int encoder, PIDController controller) {
        this.directionMotor = new TalonFX(directionMotor);
        this.driveMotor = new TalonFX(driveMotor);
        this.encoder = new CANCoder(encoder);
        this.angleController = controller;
    }

    public void init() {
        directionMotor.configFactoryDefault();
        directionMotor.setInverted(true);
        directionMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(true);
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
       
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);


    }

    public void disable() {
        directionMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.setNeutralMode(NeutralMode.Coast);

    }

    public double nearestAngle(double currentAngle, double targetAngle) {
        double direction = (targetAngle % (2 * pi) - (currentAngle % (2 * pi)));

        if (Math.abs(direction) > pi) {
            direction = -(Math.signum(direction) * (2 * pi)) + direction;
        }
        return direction;
    }

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

        driveMotor.set(ControlMode.PercentOutput, speed);
        directionMotor.set(ControlMode.PercentOutput, optimizedAngle);
    }

    public double getModuleSpeed() {

        return driveMotor.getSelectedSensorVelocity();
    }

    public double getModuleAngle() {

        return (encoder.getAbsolutePosition() / 180) * pi;
    }

    public double getModuleAngleDeg() {

        return encoder.getAbsolutePosition();
    }

    public void autoInit(double angle) {

        // make pid continuous on (-pi, pi)
        angleController.enableContinuousInput(-pi, pi);

        // get the current position reading of the direction encoder
        double currentAngle = (encoder.getAbsolutePosition()*(2*pi)/360);
        // SmartDashboard.putNumber("Current Angle", currentAngle);
        double optimizedAngle = angleController.calculate(currentAngle, angle);
        directionMotor.set(ControlMode.PercentOutput, optimizedAngle);
        // directionMotor.set(ControlMode.Position,radiansToTicks(2*pi));

    }

    public double getOptimizedAngle() {
        return angleController.calculate(getModuleAngleDeg(), 0);
    }

    public double getError() {
        return angleController.getPositionError();
    }

    public double getPosition() {
        return directionMotor.getSelectedSensorPosition();
    }

    public double radiansToTicks(double radians) {
        return radians * (44000 / (2 * pi));
    }

    public void autoInitDisabled(double angle) {

        // make pid continuous on (-pi, pi)
        angleController.enableContinuousInput(-pi, pi);

        // get the current position reading of the direction encoder
        double currentAngle = (encoder.getAbsolutePosition() *(2*pi) / 360);
        // SmartDashboard.putNumber("Current Angle", currentAngle);
        double optimizedAngle = angleController.calculate(currentAngle, pi / 2);
        directionMotor.set(ControlMode.PercentOutput, optimizedAngle);
        // directionMotor.set(ControlMode.Position,radiansToTicks(2*pi));
    }

    public double encoderPos() {
        return encoder.getAbsolutePosition();
    }
}
