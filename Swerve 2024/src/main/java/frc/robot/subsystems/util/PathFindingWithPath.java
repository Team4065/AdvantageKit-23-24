package frc.robot.subsystems.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PathFindingWithPath {
    public static Command pathFindingAutoBuilder(String endPath) {
        PathPlannerPath endPathTraj = PathPlannerPath.fromPathFile(endPath);

        PathConstraints constraints = new PathConstraints(
            3.0, 4.0, 
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );

        Command pathFindingCommand = AutoBuilder.pathfindThenFollowPath(endPathTraj, constraints, 3);
        return pathFindingCommand.until(() -> !(RobotContainer.BB.getAsBoolean()));
    } 
}
