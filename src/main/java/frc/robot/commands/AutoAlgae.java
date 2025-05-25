package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Game.AlgaePosition;
import frc.robot.RobotContainer;
import frc.robot.utils.CommandUtils;

public class AutoAlgae extends SequentialCommandGroup {

  private boolean alignTimeout = false;
  private static boolean cancelCommand = false;
  private static boolean isRunning = false;

  public static void performCancel() {
    cancelCommand = true;
  }

  public static boolean isRunning() {
    return isRunning;
  }

  public static void stopRunning() {
    isRunning = false;
  }

  public AutoAlgae(AlgaePosition reefPole) {
    addCommands(
        new InstantCommand(
            () -> {
              alignTimeout = false;
              cancelCommand = false;
              isRunning = true;
            }),
        CommandUtils.finishOnInterrupt(
                AlgaeAlign.alignTarget(reefPole, false)
                    .handleInterrupt(() -> alignTimeout = true) // align until inturupted
                    .withTimeout(2.5)
                    .asProxy())
            .alongWith(
                new WaitUntilCommand(
                        () -> {
                          Pose2d pose = reefPole.getPose();

                          double clearanceMin = 0.2;
                          double clearanceMax = 0.3;

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
                        ElevatorMoveThenAlgaeGrab.create(reefPole.getLevel().getHeight(), true)
                            .asProxy()))
            .onlyWhile(() -> RobotState.isAutonomous() || !cancelCommand),
        new WaitUntilCommand(
            () -> {
              Pose2d pose = reefPole.getPose();

              double dist =
                  RobotContainer.poseSensorFusion
                      .getEstimatedPosition()
                      .getTranslation()
                      .getDistance(pose.getTranslation());

              return cancelCommand || dist > 0.4;
            }),
        new ElevatorMoveThenAlgaeGrabEnd(reefPole.getLevel().getHeight(), true).asProxy());
  }
}
