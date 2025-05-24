package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.hybrid.AlignToPose;
import frc.robot.dashboard.DashboardUI;
import java.util.function.Supplier;

public class GameAlign {
  private static boolean wasInterrupted = false;

  public static Command alignTarget(
      Supplier<Pose2d> target, Transform2d pathTargetTransform, boolean repeatedly) {
    Pose2d targetPose = target.get();

    Pose2d pathTarget = targetPose.transformBy(pathTargetTransform);

    Command alignCmd =
        new AlignToPose(
            () -> {
              return target.get();
            });

    if (repeatedly) alignCmd = alignCmd.repeatedly();

    return new SequentialCommandGroup(
            new InstantCommand(() -> wasInterrupted = false),
            PathAlign.create(pathTarget),
            alignCmd)
        .raceWith(
            new WaitUntilCommand(
                    () -> {
                      if (RobotState.isAutonomous()) return false;

                      if (AlignToPose.getDrivetrainControl() == null) return false;

                      var driverVelocity =
                          DashboardUI.Overview.getControl()
                              .getDrivetrainControl()
                              .getDriverVelocity();
                      var targetVelocity = AlignToPose.getDrivetrainControl().getTargetVelocity();

                      // If the driver is moving in the opposite direction of the target velocity,
                      // interrupt
                      if (Math.signum(driverVelocity.getX()) != 0
                          && Math.signum(driverVelocity.getX())
                              != Math.signum(targetVelocity.getX())
                          && Math.abs(driverVelocity.getX() - targetVelocity.getX()) > 2.5) {
                        return true;
                      }

                      if (Math.signum(driverVelocity.getY()) != 0
                          && Math.signum(driverVelocity.getY())
                              != Math.signum(targetVelocity.getY())
                          && Math.abs(driverVelocity.getY() - targetVelocity.getY()) > 2.5) {
                        return true;
                      }

                      if (Math.signum(driverVelocity.getRotation().getRadians()) != 0
                          && Math.signum(driverVelocity.getRotation().getRadians())
                              != Math.signum(targetVelocity.getRotation().getRadians())
                          && Math.abs(
                                  driverVelocity.getRotation().getRadians()
                                      - targetVelocity.getRotation().getRadians())
                              > Units.degreesToRadians(20)) {
                        return true;
                      }

                      return false;
                    })
                .andThen(() -> wasInterrupted = true));
  }

  public static boolean wasInterrupted() {
    return wasInterrupted;
  }
}
