package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.RobotAlignPose;
import frc.robot.RobotContainer;
import frc.robot.commands.Align;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.commands.CoralShoot;
import frc.robot.utils.CommandUtils;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class BargeLeftAuto extends SequentialCommandGroup {

  private Command createSource(String reefLetter)
      throws FileVersionException, IOException, ParseException {
    return new CoralIntakeFromSource(false)
        .beforeStarting(new WaitCommand(0.2))
        .alongWith(
            Commands.either(
                    AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile(
                                "Reef" + reefLetter + "ToSourceLeftOuterNoElevator"))
                        .andThen(
                            CommandUtils.finishOnInterrupt(
                                Align.create(0.01, 0.02, true, 1.5).withTimeout(0.6)))
                        .andThen(new InstantCommand(() -> RobotContainer.drivetrain.kill()))
                        .andThen(new WaitCommand(0.6))
                        .andThen(
                            AutoBuilder.followPath(
                                PathPlannerPath.fromPathFile("SourceLeftOuterToElevatorStart"))),
                    AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile("ElevatorStartToSourceLeftOuter"))
                        .andThen(
                            CommandUtils.finishOnInterrupt(
                                Align.create(0.01, 0.02, true, 1.5).withTimeout(0.6)))
                        .andThen(new InstantCommand(() -> RobotContainer.drivetrain.kill()))
                        .andThen(new WaitCommand(0.6))
                        .andThen(
                            AutoBuilder.followPath(
                                PathPlannerPath.fromPathFile("SourceLeftOuterToElevatorStart"))),
                    () ->
                        RobotAlignPose.closestReefTo(
                                RobotContainer.poseTracker.getEstimatedPosition(), 0.7)
                            != null)
                .repeatedly()
                .onlyWhile(() -> !RobotContainer.elevatorHead.hasCoral()));
  }

  public BargeLeftAuto() throws FileVersionException, IOException, ParseException {
    addCommands(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeLeftToL4")),
        CommandUtils.finishOnInterrupt(Align.create(0.01, 0.02, true, 1.5).withTimeout(1.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        createSource("J"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefK")),
        CommandUtils.finishOnInterrupt(Align.create(0.01, 0.02, true, 1.5).withTimeout(1.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        createSource("K"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefL")),
        CommandUtils.finishOnInterrupt(Align.create(0.01, 0.02, true, 1.5).withTimeout(1.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefLToPark")),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()));
    addRequirements(RobotContainer.drivetrain);
  }
}
