package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public interface IntakeRollersIO {
    @AutoLog
    public static class IntakeIORollerInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setRollerDutyCycle(
            double staticPercentTop,
            double staticPercentBottom,
            double manualIn,
            double manualOut,
            boolean lightStop) {
    }

     public default void stop() {
    }
}
