// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  DrivetrainIO io;
  DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);

  /** Creates a new Drivetrain. */
  public Drivetrain(DrivetrainIO io) {
    this.io = io;
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateIOInputs(inputs);
    Logger.getInstance().processInputs("Drivetrain", inputs);

    odometry.update(new Rotation2d(inputs.gyroYaw), getLeftPositionMeters(), getRightPositionMeters());
    Logger.getInstance().recordOutput("Odometery", getPose());
  }

  /* @Override 
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    io.updateIOInputs(inputs);
    Logger.getInstance().processInputs("Drivetrain", inputs);

    odometry.update(new Rotation2d(inputs.gyroYaw), getLeftPositionMeters(), getRightPositionMeters());
    Logger.getInstance().recordOutput("Odometery", getPose());
  } */

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    var speeds = DifferentialDrive.tankDriveIK(leftSpeed, rightSpeed, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
  }
  
  public void stop() {
    io.setVoltage(0, 0);
  }

  public double getLeftPositionMeters() {
    return inputs.leftEncoderPos * Units.inchesToMeters(Constants.AutoConstants.kWheelRadiusInches);
  }

  public double getRightPositionMeters() {
    return inputs.rightEncoderPos * Units.inchesToMeters(Constants.AutoConstants.kWheelRadiusInches);
  }

  public double getLeftVelocityMeters() {
    return inputs.leftEncoderVelc * Units.inchesToMeters(Constants.AutoConstants.kWheelRadiusInches);
  }

  public double getRightVelocityMeters() {
    return inputs.rightEncoderVelc * Units.inchesToMeters(Constants.AutoConstants.kWheelRadiusInches);
  }
}
