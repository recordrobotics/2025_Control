package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.RobotAlignPose;
import frc.robot.RobotContainer;
import frc.robot.commands.hybrid.AlignToPose;
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
}
