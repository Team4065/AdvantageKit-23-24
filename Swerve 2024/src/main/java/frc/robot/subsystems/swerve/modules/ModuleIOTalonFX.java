package frc.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ModuleIOTalonFX implements ModuleIO {
  final TalonFX driveMotor;
  final TalonFX turnMotor;

  CANcoder encoder;

  final StatusSignal<Double> drivePos, driveVelc, driveAppliedVolts, driveCurrent;
  final StatusSignal<Double> turnAbsPos, turnPos, turnVelc, turnAppliedVolts, turnCurrent;

  final boolean turnMotorInverted = false;
  final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    /*
        0 - FRONT LEFT
        1 - FRONT RIGHT
        2 - BACK LEFT
        3 - BACK RIGHT
    */

    // Set up motors
    switch (index) {
      case 0:
        driveMotor = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontLeftDriveID);
        turnMotor = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontLeftTurnID);
        encoder = new CANcoder(Constants.SwerveConstants.ModuleConstants.frontLeftCanCoderID);
        absoluteEncoderOffset = Constants.SwerveConstants.ModuleConstants.frontLeftEncoderOffset;
        break;

      case 1:
        driveMotor = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontRightDriveID);
        turnMotor = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontRightTurnID);
        encoder = new CANcoder(Constants.SwerveConstants.ModuleConstants.frontRightCanCoderID);
        absoluteEncoderOffset = Constants.SwerveConstants.ModuleConstants.frontRightEncoderOffset;
        break;

      case 2:
        driveMotor = new TalonFX(Constants.SwerveConstants.ModuleConstants.backLeftDriveID);
        turnMotor = new TalonFX(Constants.SwerveConstants.ModuleConstants.backLeftTurnID);
        encoder = new CANcoder(Constants.SwerveConstants.ModuleConstants.backLeftCanCoderID);
        absoluteEncoderOffset = Constants.SwerveConstants.ModuleConstants.backLeftEncoderOffset;
        break;

      case 3:
        driveMotor = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontRightDriveID);
        turnMotor = new TalonFX(Constants.SwerveConstants.ModuleConstants.frontRightTurnID);
        encoder = new CANcoder(Constants.SwerveConstants.ModuleConstants.frontRightCanCoderID);
        absoluteEncoderOffset = Constants.SwerveConstants.ModuleConstants.frontRightEncoderOffset;
        break;

      default:
        throw new RuntimeException("INVALID MODULE SELECTED");
    }

    // Configure motor current limits and brake mode
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveMotor.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 40;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnMotor.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    encoder.getConfigurator().apply(new CANcoderConfiguration());

    // Set sensor values
    drivePos = driveMotor.getPosition();
    driveVelc = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getStatorCurrent();

    turnAbsPos = encoder.getAbsolutePosition();
    turnPos = turnMotor.getPosition();
    turnVelc = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, drivePos, turnPos); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0,
      driveVelc,
      driveAppliedVolts,
      driveCurrent,
      turnAbsPos,
      turnVelc,
      turnAppliedVolts,
      turnCurrent
    );

    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    BaseStatusSignal.refreshAll(
      drivePos,
      driveVelc,
      driveAppliedVolts,
      driveCurrent,
      turnAbsPos,
      turnPos,
      turnVelc,
      turnAppliedVolts,
      turnCurrent
    );

    inputs.drivePositionRad =
        Units.rotationsToRadians(
            drivePos.getValueAsDouble()
              / Constants.SwerveConstants.DRIVE_GEAR_RATIO);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(
            driveVelc.getValueAsDouble()
              / Constants.SwerveConstants.DRIVE_GEAR_RATIO);
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsPos.getValueAsDouble()).minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
            turnPos.getValueAsDouble()
                / Constants.SwerveConstants.encoderTicks
                / Constants.SwerveConstants.TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.radiansToRotations(
            turnVelc.getValueAsDouble()
                / Constants.SwerveConstants.encoderTicks
                / Constants.SwerveConstants.TURN_GEAR_RATIO);
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
  }

  // TO-DO: finish implemeting default methods
  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveMotor.setInverted(false);
    var config = new MotorOutputConfigs();
    if (enable) {
      config.NeutralMode = NeutralModeValue.Brake;
    } else {
      config.NeutralMode = NeutralModeValue.Coast;
    }

    driveMotor.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    if (turnMotorInverted) {
      turnMotor.setInverted(false);
    } else {
      turnMotor.setInverted(true);
    }

    var config = new MotorOutputConfigs();
    if (enable) {
      config.NeutralMode = NeutralModeValue.Brake;
    } else {
      config.NeutralMode = NeutralModeValue.Coast;
    }

    turnMotor.getConfigurator().apply(config);
  }
}
