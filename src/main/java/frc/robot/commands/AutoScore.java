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
import frc.robot.control.AbstractControl.ReefLevelSwitchValue;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.ElevatorHead.GamePiece;
import java.util.Set;
import java.util.function.Supplier;

public class AutoScore extends SequentialCommandGroup {

    private Pose2d backawayTargetPose = null;
    private boolean isL1 = false;

    private CoralLevel getLevel() {
        return isL1 && !RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.CORAL)
                ? CoralLevel.L1
                : DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel();
    }

    @SuppressWarnings("unchecked")
    public AutoScore(CoralPosition reefPole) {
        addCommands(
                new InstantCommand(() -> {
                    isL1 = DashboardUI.Overview.getControl().getReefLevelSwitchValue() == ReefLevelSwitchValue.L1;
                }),
                WaypointAlign.alignWithCommand(
                        ReefAlign.generateWaypoints(() -> reefPole, this::getLevel, false),
                        // 8s timeout for first waypoint, 4s for second
                        new Double[] {8.0, 4.0},
                        // start elevator immediately
                        -1,
                        // elevator has to be fully extended before moving to second waypoint
                        0,
                        Commands.either(
                                new DeferredCommand(
                                        () -> new ElevatorMove(DashboardUI.Overview.getControl()
                                                        .getReefLevelSwitchValue()
                                                        .toCoralLevel()
                                                        .getHeight())
                                                .asProxy(),
                                        Set.of()),
                                new CoralIntakeMoveL1().asProxy(),
                                () -> getLevel() != CoralLevel.L1)),
                // if ruckig timed out, wait until autoscore is pressed again
                new WaitUntilCommand(() -> DashboardUI.Overview.getControl().getAutoScore())
                        .onlyIf(() -> !RuckigAlign.lastAlignSuccessful() && !RobotState.isAutonomous()),
                        new WaitUntilCommand(() -> RobotState.isAutonomous()
                                        || !DashboardUI.Overview.getControl().getAutoScore())
                                .andThen(Commands.either(
                                        new CoralShoot()
                                                .andThen(() -> backawayTargetPose = RobotContainer.poseSensorFusion
                                                        .getEstimatedPosition()
                                                        .transformBy(new Transform2d(-0.6, 0, Rotation2d.kZero)))
                                                .andThen(WaypointAlign.align(
                                                                new Supplier[] {() -> backawayTargetPose},
                                                                new Boolean[] {true},
                                                                new Double[] {2.0}) // back away
                                                        .alongWith(new DeferredCommand(
                                                                        () -> new WaitCommand(
                                                                                RobotContainer.elevator
                                                                                                        .getNearestHeight()
                                                                                                == ElevatorHeight.L4
                                                                                        ? 1.0
                                                                                        : 0),
                                                                        Set.of())
                                                                .andThen(
                                                                        new ElevatorMove(ElevatorHeight.BOTTOM)
                                                                                .asProxy())
                                                                .onlyIf(() -> !CoralShoot.failedToShoot))),
                                        new CoralIntakeShootL1().asProxy(),
                                        () -> getLevel() != CoralLevel.L1)));
    }
}
