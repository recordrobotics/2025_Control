package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotAlignPose;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.commands.CoralShoot;
import frc.robot.commands.ReefAlign;
import frc.robot.utils.CommandUtils;
import frc.robot.utils.RepeatConditionallyCommand;
import java.io.IOException;
import java.util.Set;
import org.json.simple.parser.ParseException;

public class BargeRightAuto extends SequentialCommandGroup {

  private static final double SOURCE_TIMEOUT = 0.8;
  private static double sourceStart;

  private Command sourceWait() {
    return new DeferredCommand(
        () -> {
          double time = Timer.getTimestamp() - sourceStart;
          return new WaitCommand(time > SOURCE_TIMEOUT ? 0 : SOURCE_TIMEOUT - time);
        },
        Set.of());
  }

  private Command alignWithVision() { // TODO use RobotContainer.hasVision when it is tested
    return new RepeatConditionallyCommand(
        ReefAlign.alignClosest(),
        () ->
            !(RobotContainer.poseSensorFusion.getLeftCamera().hasVision()
                || RobotContainer.poseSensorFusion.getCenterCamera().hasVision()),
        true);
  }

  private Command createSource(String reefLetter)
      throws FileVersionException, IOException, ParseException {
    return new CoralIntakeFromSource(false)
        .beforeStarting(new WaitCommand(0.3))
        .alongWith(
            Commands.either(
                    AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile(
                                "Reef" + reefLetter + "ToSourceRightOuterNoElevator"))
                        .andThen(
                            CommandUtils.finishOnInterrupt(
                                ReefAlign.alignClosest()
                                    .withTimeout(0.1)
                                    .beforeStarting(() -> sourceStart = Timer.getTimestamp())))
                        .andThen(new InstantCommand(() -> RobotContainer.drivetrain.kill()))
                        .andThen(sourceWait())
                        .andThen(
                            AutoBuilder.followPath(
                                PathPlannerPath.fromPathFile("SourceRightOuterToElevatorStart"))),
                    AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile("ElevatorStartToSourceRightOuter"))
                        .andThen(
                            CommandUtils.finishOnInterrupt(
                                ReefAlign.alignClosest()
                                    .withTimeout(0.1)
                                    .beforeStarting(() -> sourceStart = Timer.getTimestamp())))
                        .andThen(new InstantCommand(() -> RobotContainer.drivetrain.kill()))
                        .andThen(sourceWait())
                        .andThen(
                            AutoBuilder.followPath(
                                PathPlannerPath.fromPathFile("SourceRightOuterToElevatorStart"))),
                    () ->
                        RobotAlignPose.closestReefTo(
                                RobotContainer.poseSensorFusion.getEstimatedPosition(), 0.7)
                            != null)
                .repeatedly()
                .onlyWhile(() -> !RobotContainer.elevatorHead.hasCoral()));
  }

  public BargeRightAuto() throws FileVersionException, IOException, ParseException {
    addCommands(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeRightToL4")),
        CommandUtils.finishOnInterrupt(alignWithVision().withTimeout(1.5)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        createSource("E"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefD")),
        CommandUtils.finishOnInterrupt(alignWithVision().withTimeout(1.5)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        createSource("D"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefC")),
        CommandUtils.finishOnInterrupt(alignWithVision().withTimeout(1.5)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefCToPark")),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()));
    addRequirements(RobotContainer.drivetrain);
  }
}
