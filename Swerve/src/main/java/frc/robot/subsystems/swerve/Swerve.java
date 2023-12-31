// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleIO;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {

  static final double max_drive_speed = Units.feetToMeters(Constants.SwerveConstants.MAX_SPEED_FEET);
  static final double track_width_x = Units.inchesToMeters(30);
  static final double track_width_y = Units.inchesToMeters(30);
  static final double drivetrain_radius = Math.hypot(track_width_x / 2, track_width_y / 2);
  static final double max_angular_speed = max_drive_speed / drivetrain_radius;

  final GyroIO gyroIO;
  final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  final Module[] modules = new Module[4];

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  Pose2d pose = new Pose2d(0, 0, new Rotation2d());
  Rotation2d lastGyroRotation = new Rotation2d();

  /** Creates a new Swerve. */
  public Swerve(GyroIO gyroIO, ModuleIO moduleFL, ModuleIO moduleFR, ModuleIO moduleBL, ModuleIO moduleBR) {
    this.gyroIO = gyroIO; // Initialize gyro
    // Initialize modules
    modules[0] = new Module(moduleFL, 0);
    modules[1] = new Module(moduleFR, 1);
    modules[2] = new Module(moduleBL, 2);
    modules[3] = new Module(moduleBR, 3);

    // Configure PathPlanner
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::setPose,
      () -> kinematics.toChassisSpeeds(getModuleStates()),
      this::runVelc,
      new HolonomicPathFollowerConfig(
        max_drive_speed,
        drivetrain_radius,
        new ReplanningConfig()
      ),
      this
    );   
    
    PathPlannerLogging.setLogActivePathCallback(
      (activePath) -> {
        Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
      }
    );

  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    // Call periodic method of each module
    for (var module : modules) {
      module.periodic();
    }

    // Stop motors if robot is disabled, VERY IMPORTANT DO NOT REMOVE
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log measured states
    Logger.recordOutput("SwerveStates/Measured", getModuleStates());

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometery
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules[i].getPosDelta();
    }

    // Twist is the motion of the robot since it was last updated
    // Estimates the x, y, theta in simulation without gyro

    var twist = kinematics.toTwist2d(wheelDeltas);
    if (gyroInputs.isConnected) {
      twist = new Twist2d(twist.dx, twist.dy, gyroInputs.yawPos.minus(lastGyroRotation).getRadians());

      lastGyroRotation = gyroInputs.yawPos;
    } else {
      
    }

    pose = pose.exp(twist);
    Logger.recordOutput("Odometry/Robot", getPose());
  }

  public SwerveModulePosition[] getModulePos() {
    SwerveModulePosition[] modulePos = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      modulePos[i] = modules[i].getPosDelta();
    }
    
    return modulePos;
  }

  // Your field-relative control

  public void runVelc(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, max_drive_speed);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetPoint(setpointStates[i]);
    }
    // The OptimizedSetpoints and Setpoints fields measure the same values

    // OptimizedSetpoints fields are used to visualize the swerve as a whole, uses a specified array
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates); 
    // Setpoints fields are used to see indivudal setpoints, uses an unspecified array
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public double getMaxLinearSpeed() {    
    return max_drive_speed;
  }

  public double getMaxAngularSpeed() {
    return max_angular_speed;
  }

  // Stop robot
  public void stop() {
    runVelc(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // Aigns all modules to the front and basically acts like a tank drive
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelc();
    }
    return driveVelocityAverage / 4.0;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  // Return robot position
  public Pose2d getPose() {
    return pose;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  // Get rotation
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(track_width_x / 2.0, track_width_y / 2.0),
      new Translation2d(track_width_x / 2.0, -track_width_y / 2.0),
      new Translation2d(-track_width_x / 2.0, track_width_y / 2.0),
      new Translation2d(-track_width_x / 2.0, -track_width_y / 2.0)
    };
  }
}
