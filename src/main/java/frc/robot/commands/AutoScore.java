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
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.hybrid.AlignToPose;
import frc.robot.control.AbstractControl.ReefLevelSwitchValue;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.utils.CommandUtils;
import java.util.Set;

public class AutoScore extends SequentialCommandGroup {

  private Pose2d backawayTargetPose = null;
  private boolean alignTimeout = false;
  private boolean isL1 = false;

  private CoralLevel getLevel() {
    return isL1 && !RobotContainer.elevatorHead.hasCoral()
        ? CoralLevel.L1
        : DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel();
  }

  public AutoScore(CoralPosition reefPole) {
    addCommands(
        new InstantCommand(
            () -> {
              alignTimeout = false;
              isL1 =
                  DashboardUI.Overview.getControl().getReefLevelSwitchValue()
                      == ReefLevelSwitchValue.L1;
            }),
        CommandUtils.finishOnInterrupt(
                ReefAlign.alignTarget(reefPole, this::getLevel, false)
                    .handleInterrupt(() -> alignTimeout = true) // align until inturupted
                    .withTimeout(2.5)
                    .asProxy())
            .alongWith(
                new WaitUntilCommand(
                        () -> {
                          var level = getLevel();
                          Pose2d pose = reefPole.getPose(level);

                          double clearanceMin = (level == CoralLevel.L1 ? 0.3 : 0.2);
                          double clearanceMax = (level == CoralLevel.L1 ? 0.5 : 0.3);

                          double dist =
                              RobotContainer.poseSensorFusion
                                  .getEstimatedPosition()
                                  .getTranslation()
                                  .getDistance(pose.getTranslation());

                          return (dist < clearanceMax && dist > clearanceMin)
                              || GameAlign.wasInterrupted()
                              || alignTimeout;
                        })
                    .andThen(
                        Commands.either(
                            new DeferredCommand(
                                () ->
                                    new ElevatorMove(
                                            DashboardUI.Overview.getControl()
                                                .getReefLevelSwitchValue()
                                                .toCoralLevel()
                                                .getHeight())
                                        .asProxy(),
                                Set.of()),
                            new CoralIntakeMoveL1().asProxy(),
                            () -> getLevel() != CoralLevel.L1)))
            .onlyWhile(
                () ->
                    RobotState.isAutonomous() || DashboardUI.Overview.getControl().getAutoScore()),
        Commands.either(
            new WaitUntilCommand(
                    () ->
                        RobotState.isAutonomous()
                            || !DashboardUI.Overview.getControl().getAutoScore())
                .andThen(
                    Commands.either(
                        new CoralShoot()
                            .andThen(
                                () ->
                                    backawayTargetPose =
                                        RobotContainer.poseSensorFusion
                                            .getEstimatedPosition()
                                            .transformBy(
                                                new Transform2d(-0.3, 0, Rotation2d.kZero)))
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
                                                new ElevatorMove(ElevatorHeight.BOTTOM)
                                                    .asProxy()))),
                        new CoralIntakeShootL1().asProxy(),
                        () -> getLevel() != CoralLevel.L1)),
            Commands.either(
                new ElevatorMove(ElevatorHeight.BOTTOM).asProxy(),
                new InstantCommand(
                        () -> {
                          RobotContainer.coralIntake.set(CoralIntakeState.UP);
                        },
                        RobotContainer.coralIntake)
                    .asProxy(),
                () -> getLevel() != CoralLevel.L1),
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
