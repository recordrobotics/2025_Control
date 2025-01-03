package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldPosition;
import frc.robot.commands.subroutines.PushSpeaker;
import frc.robot.commands.subroutines.SetupSpeaker;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TriggerProcessor.TriggerDistance;

@TriggerDistance(distance = 3, position = FieldPosition.Speaker)
public class TeleSpeakerScore extends SequentialCommandGroup {

  // Load the path we want to pathfind to and follow
  private PathPlannerPath path = PathPlannerPath.fromPathFile("PathFindSpeaker");

  // Create the constraints to use while pathfinding. The constraints defined in the path will only
  // be used for the path.
  // docs are here:
  // http://gabybot.com/RobotCoreDoc/classcom_1_1pathplanner_1_1lib_1_1path_1_1_path_constraints.html
  private PathConstraints constraints =
      new PathConstraints(
          0.5, // Max velocity meters per second
          4.0, // Max acceleration meters per second per second
          Units.degreesToRadians(540), // Max angular velocity radians per second
          Units.degreesToRadians(720)); // Max angular acceleration radians per second per second

  private static Channel _channel;
  private static Shooter _shooter;

  public TeleSpeakerScore(Channel channel, Shooter shooter) {

    _channel = channel;
    _shooter = shooter;

    addCommands(
        new SetupSpeaker(_shooter),
        AutoBuilder.pathfindThenFollowPath(path, constraints, 0),
        new PushSpeaker(_channel, _shooter));
  }
}
