package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotAlignPose;
import frc.robot.RobotContainer;
import frc.robot.commands.hybrid.AlignToPose;
import frc.robot.control.AbstractControl.AutoScoreDirection;
import java.util.Set;

public class Align {
  public static Command create(double tolerance, double rotTol, boolean forceUseTranslation) {
    return new DeferredCommand(
        () -> {
          RobotAlignPose alignPose =
              RobotAlignPose.closestTo(RobotContainer.poseTracker.getEstimatedPosition(), 1);
          if (alignPose == null) return Commands.none();

          return new AlignToPose(
              alignPose.getPose(),
              tolerance,
              rotTol,
              forceUseTranslation || alignPose.useTranslation());
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command createWithTimeout(
      double tolerance, double rotTol, boolean forceUseTranslation, double timeout) {
    return new DeferredCommand(
        () -> {
          RobotAlignPose alignPose =
              RobotAlignPose.closestTo(RobotContainer.poseTracker.getEstimatedPosition(), 1);
          if (alignPose == null) return Commands.none();

          return new WaitCommand(timeout)
              .deadlineFor(
                  new AlignToPose(
                      alignPose.getPose(),
                      tolerance,
                      rotTol,
                      forceUseTranslation || alignPose.useTranslation()));
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command createForReef(
      AutoScoreDirection direction, double tolerance, double rotTol) {
    return new DeferredCommand(
        () -> {
          Pose2d pose = RobotContainer.poseTracker.getEstimatedPosition();

          double maxDistance = 1;
          RobotAlignPose closest = null;
          double closestDistance = Double.MAX_VALUE;
          for (RobotAlignPose align :
              direction == AutoScoreDirection.Left
                  ? RobotAlignPose.leftReefPoses
                  : RobotAlignPose.rightReefPoses) {
            double distance = align.getPose().getTranslation().getDistance(pose.getTranslation());
            if (distance <= maxDistance && distance < closestDistance) {
              closest = align;
              closestDistance = distance;
            }
          }

          if (closest == null) return Commands.none();

          return new AlignToPose(closest.getPose(), tolerance, rotTol, true);
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command createForReefBackaway(
      AutoScoreDirection direction, double tolerance, double rotTol) {
    return new DeferredCommand(
        () -> {
          Pose2d pose = RobotContainer.poseTracker.getEstimatedPosition();

          double maxDistance = 1;
          RobotAlignPose closest = null;
          double closestDistance = Double.MAX_VALUE;
          for (RobotAlignPose align :
              direction == AutoScoreDirection.Left
                  ? RobotAlignPose.leftReefPoses
                  : RobotAlignPose.rightReefPoses) {
            double distance = align.getPose().getTranslation().getDistance(pose.getTranslation());
            if (distance <= maxDistance && distance < closestDistance) {
              closest = align;
              closestDistance = distance;
            }
          }

          if (closest == null) return Commands.none();

          Pose2d backawayPose =
              closest.getPose().transformBy(new Transform2d(-0.5, 0, Rotation2d.kZero));

          return new AlignToPose(backawayPose, tolerance, rotTol, true);
        },
        Set.of(RobotContainer.drivetrain));
  }
}
