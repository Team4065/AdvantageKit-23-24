package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants;
import frc.robot.Constants.DiffDriveConstants;

public class DriveSimIO implements DrivetrainIO {
  DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(1),
    Constants.DiffDriveConstants.AutoConstants.kGearRatio,
    7.5,
    60.0,
    Units.inchesToMeters(Constants.DiffDriveConstants.AutoConstants.kWheelRadiusInches),
    Constants.DiffDriveConstants.AutoConstants.kTrackWidthMeters,
    null
  );

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  @Override
  public void updateIOInputs(DrivetrainIOInputs inputs) {
    sim.update(0.02);
    inputs.leftEncoderPos = sim.getLeftPositionMeters() / Units.inchesToMeters(Constants.DiffDriveConstants.AutoConstants.kWheelRadiusInches);
    inputs.leftEncoderVelc = sim.getLeftVelocityMetersPerSecond() / Units.inchesToMeters(Constants.DiffDriveConstants.AutoConstants.kWheelRadiusInches);
    inputs.leftAppliedVolts = leftAppliedVolts;

    inputs.rightEncoderPos = sim.getRightPositionMeters() / Units.inchesToMeters(Constants.DiffDriveConstants.AutoConstants.kWheelRadiusInches);
    inputs.rightEncoderVelc = sim.getRightVelocityMetersPerSecond() / Units.inchesToMeters(Constants.DiffDriveConstants.AutoConstants.kWheelRadiusInches);
    inputs.rightAppliedVolts = rightAppliedVolts;

    inputs.gyroYaw = sim.getHeading().getRadians();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
    sim.setInputs(leftAppliedVolts, rightAppliedVolts);
  }
}