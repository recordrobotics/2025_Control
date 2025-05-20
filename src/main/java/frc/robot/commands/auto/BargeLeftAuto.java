package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Auto;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.commands.CoralShoot;
import frc.robot.commands.ReefAlign;
import frc.robot.utils.CommandUtils;
import frc.robot.utils.RepeatConditionallyCommand;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class BargeLeftAuto extends SequentialCommandGroup {

  private Command alignWithVision() { // TODO use RobotContainer.hasVision when it is tested
    return new RepeatConditionallyCommand(
        ReefAlign.alignClosest(false),
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
                                "Reef" + reefLetter + "ToSourceLeftOuterNoElevator"))
                        .andThen(new WaitCommand(Auto.SOURCE_TIMEOUT))
                        .andThen(
                            AutoBuilder.followPath(
                                PathPlannerPath.fromPathFile("SourceLeftOuterToElevatorStart"))),
                    AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile("ElevatorStartToSourceLeftOuter"))
                        .andThen(new WaitCommand(Auto.SOURCE_TIMEOUT))
                        .andThen(
                            AutoBuilder.followPath(
                                PathPlannerPath.fromPathFile("SourceLeftOuterToElevatorStart"))),
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

  public BargeLeftAuto() throws FileVersionException, IOException, ParseException {
    addCommands(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeLeftToL4")),
        CommandUtils.finishOnInterrupt(alignWithVision().withTimeout(2.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        createSource("J"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefK")),
        CommandUtils.finishOnInterrupt(alignWithVision().withTimeout(2.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        createSource("K"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefL")),
        CommandUtils.finishOnInterrupt(alignWithVision().withTimeout(2.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefLToPark")),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()));
    addRequirements(RobotContainer.drivetrain);
  }
}
