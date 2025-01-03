package frc.robot.commands.hybrid;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldPosition;
import frc.robot.commands.subroutines.PushAmp;
import frc.robot.commands.subroutines.SetupAmp;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TriggerProcessor.TriggerDistance;

@TriggerDistance(distance = 3, position = FieldPosition.Amp)
public class TeleAmpScore extends SequentialCommandGroup {

    // Load the path we want to pathfind to and follow
    private PathPlannerPath path = PathPlannerPath.fromPathFile("PathFindAmp");

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    private PathConstraints constraints = new PathConstraints(
            0.5, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    private static Channel _channel;
    private static Shooter _shooter;
    private static Crashbar _crashbar;

    public TeleAmpScore (Channel channel, Shooter shooter, Crashbar crashbar) {
        
        _channel = channel;
        _shooter = shooter;
        _crashbar = crashbar;

        addCommands(
            new SetupAmp(_shooter, _crashbar, true),
            AutoBuilder.pathfindThenFollowPath(path, constraints, 0),
            new PushAmp(_channel, _shooter, _crashbar)
        );
    }
}