package frc.robot.commands.hybrid;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.ProcessorScore;
import frc.robot.utils.TriggerProcessor.TriggerDistance;

@TriggerDistance(
    distance = Constants.HybridConstants.processorTriggerDistance,
    position = FieldPosition.Processor)
public class HybridProcessorScore extends SequentialCommandGroup {

  // Load the path we want to pathfind to and follow
  private PathPlannerPath path;

  private Alert pathNotFoundAlert = new Alert("Path Not Found: ProcessorScore", AlertType.kError);

  public HybridProcessorScore() {
    try {
      path = PathPlannerPath.fromPathFile("ProcessorScore");
    } catch (Exception e) {
      pathNotFoundAlert.set(true);
    }

    Command lightsCommand =
        RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.hybridPattern);

    addCommands(
        new ScheduleCommand(lightsCommand),
        AutoBuilder.pathfindThenFollowPath(path, Constants.HybridConstants.constraints),
        new InstantCommand(lightsCommand::cancel),
        new ProcessorScore(true));
  }
}
