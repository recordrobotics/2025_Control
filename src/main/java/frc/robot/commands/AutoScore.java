package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;

public class AutoScore extends SequentialCommandGroup {
  public AutoScore(CoralPosition reefPole) {
    addCommands(
        ReefAlign.alignTarget(
                reefPole,
                () -> DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel(),
                false,
                true) // align until inturupted
            .withTimeout(1.0)
            .asProxy()
            .alongWith(
                new WaitUntilCommand(
                        () -> {
                          var level =
                              DashboardUI.Overview.getControl()
                                  .getReefLevelSwitchValue()
                                  .toCoralLevel();
                          Pose2d pose = reefPole.getPose(level);

                          return RobotContainer.poseSensorFusion
                                  .getEstimatedPosition()
                                  .getTranslation()
                                  .getDistance(pose.getTranslation())
                              < 0.3;
                        })
                    .andThen(
                        new ElevatorMove(
                                DashboardUI.Overview.getControl()
                                    .getReefLevelSwitchValue()
                                    .toCoralLevel()
                                    .getHeight())
                            .asProxy())));
  }
}
