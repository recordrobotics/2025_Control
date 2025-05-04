package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;

public class AutoScore extends SequentialCommandGroup {
  public AutoScore(CoralPosition reefPole, CoralLevel level) {
    addCommands(
        new ElevatorMove(level.getHeight()).asProxy(),
        ReefAlign.alignTarget(reefPole, level, false, true) // align until inturupted
            .withTimeout(1.0)
            .asProxy(),
        new WaitUntilCommand(
            () -> !DashboardUI.Overview.getControl().getAutoScore()), // wait for button release
        new CoralShoot(),
        ReefAlign.alignTarget(reefPole, level, true, false) // back away
            .withTimeout(1.0)
            .finallyDo(() -> RobotContainer.drivetrain.kill())
            .asProxy()
            .andThen(
                new ElevatorMove(ElevatorHeight.BOTTOM)
                    .beforeStarting(() -> {}, RobotContainer.elevatorMoveToggleRequirement)
                    .asProxy()));
  }
}
