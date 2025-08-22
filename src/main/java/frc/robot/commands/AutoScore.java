package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import java.util.Set;

public class AutoScore extends SequentialCommandGroup {

    public AutoScore(CoralPosition reefPole, CoralLevel level) {
        addCommands(
                WaypointAlign.alignWithCommand(
                        ReefAlign.generateWaypoints(reefPole, level, false),
                        // 8s timeout for first waypoint, 4s for second
                        new Double[] {8.0, 4.0},
                        // start elevator immediately
                        -1,
                        // elevator has to be fully extended before moving to second waypoint
                        0,
                        (level != CoralLevel.L1 ? new ElevatorMove(level.getHeight()) : new CoralIntakeMoveL1())
                                .asProxy()),
                // if ruckig timed out, wait until autoscore is pressed again
                new WaitUntilCommand(() -> DashboardUI.Overview.getControl().getAutoScore())
                        .onlyIf(() -> !RuckigAlign.lastAlignSuccessful() && !RobotState.isAutonomous()),
                new WaitUntilCommand(() -> RobotState.isAutonomous()
                                || !DashboardUI.Overview.getControl().getAutoScore())
                        .andThen(
                                level != CoralLevel.L1
                                        ? new CoralShoot()
                                                .andThen(Commands.defer(
                                                                () -> WaypointAlign.align(
                                                                        RobotContainer.poseSensorFusion
                                                                                .getEstimatedPosition()
                                                                                .transformBy(
                                                                                        new Transform2d(
                                                                                                -0.6,
                                                                                                0,
                                                                                                Rotation2d.kZero)),
                                                                        2.0),
                                                                Set.of(RobotContainer.drivetrain)) // back away
                                                        .alongWith(Commands.defer(
                                                                        () -> new WaitCommand(
                                                                                RobotContainer.elevator
                                                                                                        .getNearestHeight()
                                                                                                == ElevatorHeight.L4
                                                                                        ? 1.0
                                                                                        : 0),
                                                                        Set.of())
                                                                .andThen(new ElevatorMove(ElevatorHeight.BOTTOM)
                                                                        .asProxy())
                                                                .onlyIf(() -> !CoralShoot.failedToShoot)))
                                        : new CoralIntakeShootL1().asProxy()));
    }
}
