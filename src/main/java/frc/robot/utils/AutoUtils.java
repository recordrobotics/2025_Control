package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Auto;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.commands.ReefAlign;
import frc.robot.subsystems.PoseSensorFusion.CameraTarget;
import frc.robot.subsystems.PoseSensorFusion.VisionDebouncer;
import java.io.IOException;
import java.util.Set;
import org.json.simple.parser.ParseException;

public class AutoUtils {
  public static Command alignWithVision() {
    return new DeferredCommand(
        () -> {
          int[] visionTagTarget = {
            IGamePosition.closestTo(
                    RobotContainer.poseSensorFusion.getEstimatedPosition(), CoralPosition.values())
                .apriltagId
          };

          VisionDebouncer visionDebouncer =
              RobotContainer.poseSensorFusion.registerVisionCheck(
                  CameraTarget.Elevator,
                  visionTagTarget,
                  0.5,
                  DebounceType.kFalling); // says if either LL has seen the tag in the last 0.5s

          return new RepeatConditionallyCommand(
                  ReefAlign.alignClosest(false), () -> !visionDebouncer.hasVision(), true)
              .finallyDo(() -> RobotContainer.poseSensorFusion.releaseVisionCheck(visionDebouncer));
        },
        Set.of(RobotContainer.drivetrain));
  }

  public static Command createSource(String reefLetter, String sourceSide)
      throws FileVersionException, IOException, ParseException {
    return new CoralIntakeFromSource(false)
        .beforeStarting(new WaitCommand(0.3))
        .alongWith(
            Commands.either(
                    AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile(
                                "Reef" + reefLetter + "ToSource" + sourceSide + "OuterNoElevator"))
                        .andThen(new WaitCommand(Auto.SOURCE_TIMEOUT))
                        .andThen(
                            AutoBuilder.followPath(
                                PathPlannerPath.fromPathFile(
                                    "Source" + sourceSide + "OuterToElevatorStart"))),
                    AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile(
                                "ElevatorStartToSource" + sourceSide + "Outer"))
                        .andThen(new WaitCommand(Auto.SOURCE_TIMEOUT))
                        .andThen(
                            AutoBuilder.followPath(
                                PathPlannerPath.fromPathFile(
                                    "Source" + sourceSide + "OuterToElevatorStart"))),
                    () ->
                        IGamePosition.closestTo(
                                    RobotContainer.poseSensorFusion.getEstimatedPosition(),
                                    CoralPosition.values())
                                .getFirstStagePose()
                                .getTranslation()
                                .getDistance(
                                    RobotContainer.poseSensorFusion
                                        .getEstimatedPosition()
                                        .getTranslation())
                            < 0.7)
                .repeatedly()
                .onlyWhile(() -> !RobotContainer.elevatorHead.hasCoral()));
  }
}
