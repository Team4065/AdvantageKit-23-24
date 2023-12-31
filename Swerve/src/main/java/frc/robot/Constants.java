// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static class SwerveConstants {
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double TURN_GEAR_RATIO = 150 / 7;

    public static final int encoderTicks = 2048;

    public static final double wheel_radius_meters = Units.inchesToMeters(4);

    public static final double MAX_SPEED_FEET = 14;

    public static class ModuleConstants {
      // FRONT LEFT
      public static final int frontLeftDriveID = 10;
      public static final int frontLeftTurnID = 11;
      public static final int frontLeftCanCoderID = 12;
      public static final Rotation2d frontLeftEncoderOffset = new Rotation2d(-1.8791264651599104);

      // FRONT RIGHT
      public static final int frontRightDriveID = 7;
      public static final int frontRightTurnID = 8;
      public static final int frontRightCanCoderID = 9;
      public static final Rotation2d frontRightEncoderOffset =  new Rotation2d(3.1391382843291757 + 2.8035032879397983);

      // BACK LEFT
      public static final int backLeftDriveID = 4;
      public static final int backLeftTurnID = 5;
      public static final int backLeftCanCoderID = 6;
      public static final Rotation2d backLeftEncoderOffset = new Rotation2d(2.5494760694659355);

      // BACK RIGHT
      public static final int backRightDriveID = 1;
      public static final int backRightTurnID = 2;
      public static final int backRightCanCoderID = 3;
      public static final Rotation2d backRightEncoderOffset =  new Rotation2d(0.16260196351587786);
    }
  }
}
