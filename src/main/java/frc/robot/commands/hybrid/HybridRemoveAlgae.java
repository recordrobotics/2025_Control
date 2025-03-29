package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.FieldPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorMoveThenAlgaeGrab;
import frc.robot.utils.TriggerProcessor.TriggerDistance;

@TriggerDistance(
    distance = Constants.HybridConstants.reefTriggerDistance,
    position = FieldPosition.ReefCenter)
public class HybridRemoveAlgae extends SequentialCommandGroup {
  private Alert pathNotFoundAlert = new Alert("", AlertType.kError);
  private PathPlannerPath[] paths = new PathPlannerPath[] {};

  public HybridRemoveAlgae() {
    try {
      paths =
          new PathPlannerPath[] {
            PathPlannerPath.fromPathFile("Approach Algae AB"),
            PathPlannerPath.fromPathFile("Approach Algae CD"),
            PathPlannerPath.fromPathFile("Approach Algae EF"),
            PathPlannerPath.fromPathFile("Approach Algae GH"),
            PathPlannerPath.fromPathFile("Approach Algae IJ"),
            PathPlannerPath.fromPathFile("Approach Algae KL")
          };
    } catch (Exception e) {
      pathNotFoundAlert.setText("Path Not Found! HybridRemoveAlgae");
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
      double distance = swerve_translation.getDistance(path_translation);
      if (distance < lowestDistance) {
        lowestDistance = distance;
        shortestPath = path;
      }
    }

    ElevatorHeight algaeHeight =
        Constants.HybridConstants.isAlgaeInHighPosition.get(shortestPath.name)
            ? ElevatorHeight.HIGH_REEF_ALGAE
            : ElevatorHeight.LOW_REEF_ALGAE;

    Command lightsCommand =
        RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.removeAlgaePattern);

    addCommands(
        new ScheduleCommand(lightsCommand),
        AutoBuilder.pathfindToPose(
            shortestPath.getStartingHolonomicPose().get(), Constants.HybridConstants.constraints),
        new InstantCommand(
            () -> RobotContainer.elevator.moveTo(algaeHeight),
            RobotContainer.elevator,
            RobotContainer.elevatorArm),
        AutoBuilder.followPath(shortestPath),
        new InstantCommand(lightsCommand::cancel),
        ElevatorMoveThenAlgaeGrab.create(algaeHeight, false));
  }
}
