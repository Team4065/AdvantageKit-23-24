package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
    @AutoLog
    public static class DrivetrainIOInputs {
        // Define variables that directly store values between the IO values
        public double leftEncoderPos = 0.0;
        public double rightEncoderPos = 0.0;
        public double leftEncoderVelc = 0.0;
        public double rightEncoderVelc = 0.0;
        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0;
        public double gyroYaw = 0.0;
    }

    public default void updateIOInputs(DrivetrainIOInputs inputs) {}
    public default void setVoltage(double leftVolts, double rightVolts) {}
}
