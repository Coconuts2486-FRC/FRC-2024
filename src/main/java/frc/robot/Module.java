package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

public class Module {
    private TalonFX directionMotor;

    public TalonFX driveMotor;

    public static MotorOutputConfigs coastConfig = new MotorOutputConfigs();
    public static MotorOutputConfigs brakeConfig = new MotorOutputConfigs();

    private CANcoder encoder;
    private PIDController angleController;
    private PIDController driveController;
    private double pi = Math.PI;

    private DutyCycleOut driveRequest = new DutyCycleOut(0);
    private DutyCycleOut rotateRequest = new DutyCycleOut(0);
    public Module(int directionMotor, int driveMotor, int encoder, PIDController rController,
            PIDController dController) {
        this.directionMotor = new TalonFX(directionMotor, "drive");
        this.driveMotor = new TalonFX(driveMotor, "drive");
        this.encoder = new CANcoder(encoder, "drive");
        this.angleController = rController;
        this.driveController = dController;

    }

    public void init() {
        coastConfig.withNeutralMode(NeutralModeValue.Coast);
        brakeConfig.withNeutralMode(NeutralModeValue.Brake);

        var driveMotorConfig = new MotorOutputConfigs();
        driveMotorConfig.withNeutralMode(NeutralModeValue.Brake);
        driveMotorConfig.withInverted(InvertedValue.Clockwise_Positive);

        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.getConfigurator().apply(driveMotorConfig);

        var directionMotorConfig = new MotorOutputConfigs();
        directionMotorConfig.withNeutralMode(NeutralModeValue.Brake);
        directionMotorConfig.withInverted(InvertedValue.Clockwise_Positive);

        directionMotor.getConfigurator().apply(new TalonFXConfiguration());
        directionMotor.getConfigurator().apply(directionMotorConfig);
       
    }

    public void disable(boolean button) {

        if (button) {

            directionMotor.getConfigurator().apply(coastConfig);
            driveMotor.getConfigurator().apply(coastConfig);

        } else {
            directionMotor.getConfigurator().apply(brakeConfig);
            driveMotor.getConfigurator().apply(brakeConfig);
        }
    }
    public double nearestAngle(double currentAngle, double targetAngle) {
        double direction = (targetAngle % (2 * pi) - (currentAngle % (2 * pi)));

        if (Math.abs(direction) > pi) {
            direction = -(Math.signum(direction) * (2 * pi)) + direction;
        }
        return direction;
    }
    
    public double currentAngleRadians(){
        return encoder.getAbsolutePosition().getValueAsDouble() * (2*pi);
    }
      public double currentAngleDegrees(){
        return encoder.getAbsolutePosition().getValueAsDouble() * 360;
    }


    public void drive(double speed, double angle) {
        angleController.enableContinuousInput(-pi, pi);

        double currentAngle = currentAngleRadians();
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
        rotateRequest.Output = optimizedAngle;
        directionMotor.setControl(rotateRequest);
        double optimizedSpeed = driveController.calculate(speed);
        driveRequest.Output = optimizedSpeed;
        driveMotor.setControl(driveRequest);
        // SmartDashboard.putNumber("OptimizedAngle", optimizedAngle);
    }


    public double getModuleSpeed() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Returns the angle of the module.
     * 
     * @return angle (radians)
     */
      

    /**
     * Returns the angle of the module in degrees.
     * 
     * @return angle (degrees)
     */
      

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
        double currentAngle = (encoder.getAbsolutePosition().getValueAsDouble() * (2 * pi) / 360);
        // SmartDashboard.putNumber("Current Angle", currentAngle);
        double optimizedAngle = angleController.calculate(currentAngle, angle);
        directionMotor.setControl(new DutyCycleOut(optimizedAngle));
        // directionMotor.set(ControlMode.Position,radiansToTicks(2*pi));

    }
     public double getOptimizedAngle() {
        return angleController.calculate(currentAngleDegrees(), 0);
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
        return directionMotor.getPosition().getValueAsDouble();
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
        double currentAngle = (encoder.getAbsolutePosition().getValueAsDouble() * (2 * pi) / 360);
        // SmartDashboard.putNumber("Current Angle", currentAngle);
        double optimizedAngle = angleController.calculate(currentAngle, pi / 2);
        directionMotor.setControl(new DutyCycleOut(optimizedAngle));
        // directionMotor.set(ControlMode.Position,radiansToTicks(2*pi));
    }

    /**
     * Returns the position of the module.
     */
      
    public double encoderPos() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }
}
   