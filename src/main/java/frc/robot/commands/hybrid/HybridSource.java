package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.TriggerProcessor.TriggerDistance;
import java.util.Set;

@TriggerDistance(
    distance = Constants.HybridConstants.reefTriggerDistance,
    position = FieldPosition.ReefCenter)
public class HybridSource extends SequentialCommandGroup {
  private Alert pathNotFoundAlert = new Alert("", AlertType.kError);
  private PathPlannerPath[] paths = new PathPlannerPath[] {};

  public static DeferredCommand deferred() {
    return new DeferredCommand(
        () -> new HybridSource(),
        Set.of(RobotContainer.elevator, RobotContainer.coralShooter, RobotContainer.coralIntake));
  }

  public HybridSource() {
    try {
      paths =
          new PathPlannerPath[] {
            PathPlannerPath.fromPathFile("Left Source Near"),
            PathPlannerPath.fromPathFile("Right Source Near"),
            PathPlannerPath.fromPathFile("Left Source Far"),
            PathPlannerPath.fromPathFile("Right Source Far"),
          };
    } catch (Exception e) {
      pathNotFoundAlert.setText("Path Not Found! HybridGoToSource");
      pathNotFoundAlert.set(true);
      cancel();
    }

    // Gets path closest to the robot
    PathPlannerPath shortestPath = paths[0];
    double lowestDistance = 1000;
    Translation2d swerve_translation =
        RobotContainer.poseTracker.getEstimatedPosition().getTranslation();
    for (PathPlannerPath path : paths) {
      Translation2d path_translation = path.getStartingHolonomicPose().get().getTranslation();
      if (DriverStationUtils.getCurrentAlliance() == Alliance.Red) {
        path_translation = FlippingUtil.flipFieldPosition(path_translation);
      }

      double distance = swerve_translation.getDistance(path_translation);
      if (distance < lowestDistance) {
        lowestDistance = distance;
        shortestPath = path;
      }
    }

    Command lightsCommand =
        RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.hybridPattern);

    addCommands(
        new ScheduleCommand(lightsCommand),
        AutoBuilder.pathfindThenFollowPath(shortestPath, Constants.HybridConstants.constraints)
            .asProxy(),
        new InstantCommand(lightsCommand::cancel),
        new CoralIntakeFromSource(false));
  }
}
