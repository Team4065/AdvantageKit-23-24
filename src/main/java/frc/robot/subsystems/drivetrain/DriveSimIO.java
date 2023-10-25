package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSimIO implements DrivetrainIO {
  DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(1),
    Constants.AutoConstants.kGearRatio,
    7.5,
    60.0,
    Units.inchesToMeters(Constants.AutoConstants.kWheelRadiusInches),
    Constants.AutoConstants.kTrackWidthMeters,
    null
  );

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  @Override
  public void updateIOInputs(DrivetrainIOInputs inputs) {
    sim.update(0.02);
    inputs.leftEncoderPos = sim.getLeftPositionMeters() / Units.inchesToMeters(Constants.AutoConstants.kWheelRadiusInches);
    inputs.leftEncoderVelc = sim.getLeftVelocityMetersPerSecond() / Units.inchesToMeters(Constants.AutoConstants.kWheelRadiusInches);
    inputs.leftAppliedVolts = leftAppliedVolts;

    inputs.rightEncoderPos = sim.getRightPositionMeters() / Units.inchesToMeters(Constants.AutoConstants.kWheelRadiusInches);
    inputs.rightEncoderVelc = sim.getRightVelocityMetersPerSecond() / Units.inchesToMeters(Constants.AutoConstants.kWheelRadiusInches);
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