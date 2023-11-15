package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {
  AHRS g_gyro = new AHRS(SPI.Port.kMXP);
  Rotation2d yaw = g_gyro.getRotation2d();
  double yawVelc = Units.degreesToRadians(g_gyro.getRawGyroZ());

  public GyroIONavX() {
    g_gyro.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = g_gyro.isConnected();
    inputs.yawPos = yaw;
    inputs.yawVelcRadPerSec = yawVelc;
  }
}
