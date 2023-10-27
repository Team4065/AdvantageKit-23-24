// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TankControl;
import frc.robot.subsystems.drivetrain.DriveSimIO;
import frc.robot.subsystems.drivetrain.DriveSparkMaxIO;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer { 
  public static Drivetrain drive;
  public static GenericHID controller = new XboxController(0);

  static LoggedDashboardChooser<Command> m_chooser = new LoggedDashboardChooser<>("Auto Chooser");
  static HashMap<Command, String> autoMap = new HashMap<>();

  public Command ramAutoBuilder(String pathName, HashMap<String, Command> eventMap) {
    // PathPlanner AutoBuilder, builds a full autonomous command
    RamseteAutoBuilder pathBuilder = new RamseteAutoBuilder(
        drive::getPose,
        drive::resetOdometery,
        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
        Constants.AutoConstants.kDriveKinematics,
        new SimpleMotorFeedforward(
            Constants.AutoConstants.ksVolts,
            Constants.AutoConstants.kvVoltSecondsPerMeter,
            Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
        drive::getWheelSpeeds,
        new PIDConstants(Constants.AutoConstants.kPDriveVel, 0, 0),
        drive::setVolts,
        eventMap,
        true,
        drive);
    List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName,
        PathPlanner.getConstraintsFromPath(pathName));
    final Command auto = pathBuilder.fullAuto(pathToFollow);
    autoMap.put(auto, pathName);
    return auto;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch(Constants.currentMode) {
      case REAL:
        drive = new Drivetrain(new DriveSparkMaxIO());
        break;
      case SIM:
        drive = new Drivetrain(new DriveSimIO());
        System.out.println("SIM option");
        break;
      default:
        drive = new Drivetrain(new DrivetrainIO() {});
        break;
    }

    Shuffleboard.getTab("AUTON").add(m_chooser.getSendableChooser()).withSize(3, 1);
    

    Command instantCmd = new InstantCommand();

    m_chooser.addDefaultOption("Nothing", instantCmd);
    autoMap.put(instantCmd, "nothing");
    m_chooser.addOption("Pos 1 - 2 Game Pieces", ramAutoBuilder("Red 1 - 2 GP", new HashMap<>()));
    m_chooser.addOption("Pos 1 - 2 Game Pieces & Auto Balance", ramAutoBuilder("Red 1 - 2 GP AB", new HashMap<>()));
    m_chooser.addOption("Pos 1 - 3 Game Pieces (crazy)", ramAutoBuilder("Red 1 - 3 GP", new HashMap<>()));
    m_chooser.addOption("Pos 2 - Preload & Auto Balance", ramAutoBuilder("Mid", new HashMap<>()));
    m_chooser.addOption("Pos 2 - Preload, Pickup & Auto Balance", ramAutoBuilder("Mid & Pickup", new HashMap<>()));
    m_chooser.addOption("Pos 3 - 2 Game Pieces", ramAutoBuilder("Red 3 - 2 GP", new HashMap<>()));
    m_chooser.addOption("Pos 3 - Preload & Auto Balance", ramAutoBuilder("Red 3 - 1 GP AB", new HashMap<>())); 
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(new TankControl(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.get();
  }
}
