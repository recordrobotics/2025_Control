package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.hybrid.AlignToPose;
import frc.robot.dashboard.DashboardUI;
import java.util.Set;

public class ReefAlign {
  public static Command alignClosest() {
    return new DeferredCommand(
        () -> {
          CoralPosition alignPose =
              CoralPosition.closestTo(RobotContainer.poseSensorFusion.getEstimatedPosition());

          var level = DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel();

          if (alignPose
                  .getPose(level)
                  .getTranslation()
                  .getDistance(
                      RobotContainer.poseSensorFusion.getEstimatedPosition().getTranslation())
              > Constants.Align.MAX_REEF_ALIGN_DISTANCE) return Commands.none();

          return alignTarget(alignPose, level, false, true);
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command alignTarget(
      CoralPosition pole, CoralLevel level, boolean isBackaway, boolean continueAlign) {
    Pose2d targetPose = pole.getPose(level);
    if (isBackaway) {
      targetPose = targetPose.transformBy(new Transform2d(-0.5, 0, Rotation2d.kZero));
    }

    Pose2d pathTarget = targetPose.transformBy(new Transform2d(-0.3, 0, Rotation2d.kZero));

    return new SequentialCommandGroup(
        PathAlign.createForReef(pathTarget), new AlignToPose(targetPose, true));
  }
}
