// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int leftMotorID = 1;
  public static final int rightMotorID = 2;

  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final class AutoConstants {
    public static final double ksVolts = 0.14534;
    public static final double kvVoltSecondsPerMeter = 4.0318;
    public static final double kvVoltSecondsSquaredPerMeter = 0.42035;
    public static final double kPDriveVel = 0.043262;
    
    public static final double kTrackWidthMeters = Units.inchesToMeters(24);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelMetersPerSecondSqaured = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kGearRatio = 10;
    public static final double kWheelRadiusInches = 2;

    public static final double kConversionMeters = Units.inchesToMeters(
       (1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10)
    );
 }
}