package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.RobotAlignPose;
import frc.robot.RobotContainer;
import frc.robot.commands.hybrid.AlignToPose;
import java.util.HashSet;

public class AlignToReef {
  public static Command create(double tolerance, double rotTol) {
    return new DeferredCommand(
        () -> {
          RobotAlignPose alignPose =
              RobotAlignPose.closestTo(RobotContainer.poseTracker.getEstimatedPosition(), 1);
          return new AlignToPose(
              alignPose.getPose(), tolerance, rotTol, alignPose.useTranslation());
        },
        new HashSet<>());
  }
}
