package frc.robot.commands.hybrid;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;

public class SourceAlign {
  public static Command create(boolean useTranslation) {
    return new AlignToPose(
        SourcePosition.closestTo(RobotContainer.poseTracker.getEstimatedPosition()).getPose(),
        useTranslation); // TODO
  }
}
