package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.RobotContainer;
import frc.robot.commands.hybrid.AlignToPose;
import frc.robot.dashboard.DashboardUI;
import java.util.Set;
import java.util.function.Supplier;

public class ReefAlign {
  private static boolean wasInterrupted = false;

  public static Command alignClosest(boolean repeatedly) {
    return new DeferredCommand(
        () -> {
          CoralPosition alignPose =
              IGamePosition.closestTo(
                  RobotContainer.poseSensorFusion.getEstimatedPosition(), CoralPosition.values());

          var level = DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel();

          if (alignPose
                  .getPose(level)
                  .getTranslation()
                  .getDistance(
                      RobotContainer.poseSensorFusion.getEstimatedPosition().getTranslation())
              > Constants.Align.MAX_REEF_ALIGN_DISTANCE) return Commands.none();

          return alignTarget(
              alignPose,
              () -> DashboardUI.Overview.getControl().getReefLevelSwitchValue().toCoralLevel(),
              repeatedly);
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command alignTarget(
      CoralPosition pole, Supplier<CoralLevel> level, boolean repeatedly) {
    var lvl = level.get();
    Pose2d targetPose = pole.getPose(lvl);

    Pose2d pathTarget;
    if (lvl == CoralLevel.L1) {
      pathTarget = targetPose.transformBy(new Transform2d(0, -0.3, Rotation2d.kZero));
    } else {
      pathTarget = targetPose.transformBy(new Transform2d(-0.3, 0, Rotation2d.kZero));
    }

    Command alignCmd =
        new AlignToPose(
            () -> {
              return pole.getPose(level.get());
            });

    if (repeatedly) alignCmd = alignCmd.repeatedly();

    return new SequentialCommandGroup(
            new InstantCommand(() -> wasInterrupted = false),
            PathAlign.createForReef(pathTarget),
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
