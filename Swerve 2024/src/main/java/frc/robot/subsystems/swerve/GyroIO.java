package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public class GyroIOInputs {
    public boolean isConnected = false;
    public Rotation2d yawPos = new Rotation2d();
    public double yawVelcRadPerSec = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
