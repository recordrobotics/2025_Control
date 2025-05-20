package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.hybrid.AlignToPose;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.CommandUtils;
import java.util.Set;

public class AutoScore extends SequentialCommandGroup {

  private Pose2d backawayTargetPose = null;
  private boolean alignTimeout = false;

  public AutoScore(CoralPosition reefPole) {
    addCommands(
        new InstantCommand(() -> alignTimeout = false),
        CommandUtils.finishOnInterrupt(
                ReefAlign.alignTarget(
                        reefPole,
                        () ->
                            DashboardUI.Overview.getControl()
                                .getReefLevelSwitchValue()
                                .toCoralLevel(),
                        false,
                        true)
                    .handleInterrupt(() -> alignTimeout = true) // align until inturupted
                    .withTimeout(2.5)
                    .asProxy())
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
                                  < 0.3
                              || ReefAlign.wasInterrupted()
                              || alignTimeout;
                        })
                    .andThen(
                        new DeferredCommand(
                            () ->
                                new ElevatorMove(
                                        DashboardUI.Overview.getControl()
                                            .getReefLevelSwitchValue()
                                            .toCoralLevel()
                                            .getHeight())
                                    .asProxy(),
                            Set.of())))
            .onlyWhile(
                () ->
                    RobotState.isAutonomous() || DashboardUI.Overview.getControl().getAutoScore()),
        Commands.either(
            new WaitUntilCommand(
                    () ->
                        RobotState.isAutonomous()
                            || !DashboardUI.Overview.getControl().getAutoScore())
                .andThen(
                    new CoralShoot()
                        .andThen(
                            () ->
                                backawayTargetPose =
                                    RobotContainer.poseSensorFusion
                                        .getEstimatedPosition()
                                        .transformBy(new Transform2d(-0.5, 0, Rotation2d.kZero)))
                        .andThen(
                            CommandUtils.finishOnInterrupt(
                                    new AlignToPose(() -> backawayTargetPose) // back away
                                        .withTimeout(1.0)
                                        .asProxy())
                                .finallyDo(() -> RobotContainer.drivetrain.kill())
                                .alongWith(
                                    new DeferredCommand(
                                            () ->
                                                new WaitCommand(
                                                    RobotContainer.elevator.getNearestHeight()
                                                            == ElevatorHeight.L4
                                                        ? 0.3
                                                        : 0),
                                            Set.of())
                                        .andThen(
                                            new ElevatorMove(ElevatorHeight.BOTTOM).asProxy())))),
            new ElevatorMove(ElevatorHeight.BOTTOM).asProxy(),
            () ->
                RobotState.isAutonomous()
                    || DashboardUI.Overview.getControl().getAutoScore()
                    || RobotContainer.elevator.getNearestHeight()
                        == DashboardUI.Overview.getControl()
                            .getReefLevelSwitchValue()
                            .toCoralLevel()
                            .getHeight()));
  }
}
