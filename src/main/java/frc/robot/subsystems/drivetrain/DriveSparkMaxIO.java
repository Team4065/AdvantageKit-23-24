package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;

public class DriveSparkMaxIO implements DrivetrainIO {
    // This file is mainly responsible for updating settings for SparkMaxes
    CANSparkMax leftMotor = new CANSparkMax(Constants.leftMotorID, MotorType.kBrushless);
    CANSparkMax rightMotor = new CANSparkMax(Constants.rightMotorID, MotorType.kBrushless);   

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    Gyro gyro;

    public DriveSparkMaxIO() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        gyro = new AHRS(SPI.Port.kMXP);
        gyro.calibrate();
    }

    @Override
    public void updateIOInputs(DrivetrainIOInputs inputs) {
        inputs.leftEncoderPos = Units.rotationsToRadians(leftEncoder.getPosition() / Constants.AutoConstants.kGearRatio);
        inputs.rightEncoderPos = Units.rotationsToRadians(rightEncoder.getPosition() / Constants.AutoConstants.kGearRatio);
        inputs.leftEncoderPos = Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / Constants.AutoConstants.kGearRatio);
        inputs.rightEncoderPos = Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / Constants.AutoConstants.kGearRatio);
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();

        inputs.gyroYaw = gyro.getRotation2d().getRadians() * -1;
    }

    @Override
    public void setVoltage(double leftVoltage, double rightVoltage) {
        leftMotor.setVoltage(leftVoltage);
        rightMotor.setVoltage(rightVoltage);
    }
}
